#pragma once

#include "AirBlueprintLib.h"
#include "Geom.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
//#include "CarPawnApi.h"
#include <exception>
#include <set>

extern ENGINE_API float GAverageFPS;

using namespace msr::airlib;

// One section of a path, can be either straight or arc
struct NodeDatum {
	pt2 start;  // start point, represented in meters from Unreal Engine origin
	pt2 end;    // end point
	pt2 center; // center point, only used for arc
	bool isArc; 
	double rad; // thickness in meters of line or arc, uncertainty
	double vlo; // lower velocity limit *at endpoint* in  m/s
	double vhi; // upper velocity limit *at endpoint* in m/s
	bool isCcw;

	NodeDatum& operator=(const NodeDatum & nd) {
		start = nd.start; end = nd.end; center = nd.center; isArc = nd.isArc;
		rad = nd.rad; vlo = nd.vlo, vhi = nd.vhi; isCcw = nd.isCcw;
		return *this;
	}
	double splitSize() {
		if (isArc) {
			return 40.0;
		} else {
			return 50.0;
		}
	}
	const double SLOW_LIM = 20;

	double length() {
		double EPS = 1E-5;
		if (isArc) {
			return (end - start).mag();
			/*pt2 relS = start - center;
			pt2 relE = end - center;
			double r = relS.mag();
			double thS = atan2(relS.y, relS.x), thE = atan2(relE.y, relE.x);
			double diff = (thE - thS)*((isCcw) ? -1.0 : 1.0);
			while (diff < -EPS) diff += (2.0 * M_PI);
			return diff * r;*/
		}
		else {
			return (end - start).mag();
		}
	}

	NodeDatum first() {
		double vmid = (vlo + vhi) * 0.5;
		bool slo = length() <= SLOW_LIM;
		pt2 pmid = (start + end) * 0.5;
		if (isArc) {
			pt2 relS = start - center;
			pt2 relE = end - center;
			double thS = atan2(relS.y, relS.x), thE = atan2(relE.y, relE.x);
			if (isCcw) {
				while (thE >= thS) thS += 2.0 * M_PI;
			} else {
				while (thE <= thS) thE += 2.0 * M_PI;
			}
			double thM = (thS + thE) * 0.5;
			double m = relS.mag();
			pt2 pmid = pt2( cos(thM)*m, sin(thM)*m ) + center;
			return { start, pmid, center, isArc, rad, vlo, slo ? vmid : vhi, isCcw };
		} else {
			return { start, pmid, center, isArc, rad, vlo, slo ? vmid : vhi, isCcw };
		}
	}

	NodeDatum second() {
		pt2 pmid = (start + end) * 0.5;
		if (isArc) {
			pt2 relS = start - center;
			pt2 relE = end - center;
			double thS = atan2(relS.y, relS.x), thE = atan2(relE.y, relE.x);
			if (isCcw) {
				while (thE >= thS) thS += 2.0 * M_PI;
			}
			else {
				while (thE <= thS) thE += 2.0 * M_PI;
			}
			double thM = (thS + thE) * 0.5;
			double m = relS.mag();
			pt2 pmid = pt2(cos(thM)*m, sin(thM)*m) + center;
			return { pmid, end, center, isArc, rad, vlo, vhi, isCcw };
		}
		else {
			return { pmid, end, center, isArc, rad, vlo, vhi, isCcw };
		}
	}

	
	// Feedback controller should aim for middle of speed limits
	double targetVelocity(double t) { return vlo*(1.0 - t) + vhi*t; }

	// Radius adjusted for clockwise vs. counter-clockwise
	double signedRad() {
		if (isArc) {
			double r = (center - start).mag();
			if ((center - start).isLeftOf(end - start)) {
				return r;
			}
			else {
				return -r;
			}
		}
		else {
			return std::numeric_limits<double>::infinity();
		}
	}
	 
	// Have we finished traversing this section?

	double endDist(pt2 p, pt2 d, double v) {
		double const T = 0.1; // 10Hz framerate lolol
		/* auto dvec = orientation._transformVector({ (float) 0.0, 1.0, 0.0 });
		pt2 dir2 = pt2(dvec.x(), dvec.y()).unit();
		*/
		pt2 rel = (end - p);
		pt2 base = rel.rebase(d);
		//pt2 g = pt2(0, 0) - base; // Translates to vehicle-oriented coordinates
		double dist = base.y - v * T;
		return dist;
	}
	
	bool isLeftOf(pt2 p) {
		if (isArc) {
			if (isCcw) {
				return (p - center).mag() >= (start - center).mag();
			} else {
				return (p - center).mag() <= (start - center).mag();
			}
		} else {
			return tangentAt(p, isCcw).isLeftOf(p-start);
		}
	}

	// Gets the tangent vector to the section at its *closest point* to "here", which 
	// need not actually lie on the section
	pt2 tangentAt(pt2 here, bool ccw) {
		if (isArc) {
			pt2 rel = here - center;
			pt2 relDir = rel.unit();
//			bool cw = //rel.isLeftOf(end - center);
			double delt =  ccw ? 0.5*M_PI : -0.5*M_PI;
			pt2 rot = relDir.rot(delt);
			return  rot;
		} else 
			return (end - start).unit();
	}

	// Tangent at the end of section, pointing toward start
	pt2 endTangent() {
		pt2 tan =
			  isArc 
			? ((center - end).rot(M_PI * 0.5))
			: start - end;
		return tan.unit();
	}

	// Tangent at start of section, pointing toward end.
	pt2 startTangent() {
		pt2 tan =
			isArc
			? ((start - center).rot(M_PI * 0.5))
			: end - start;
		return tan.unit();
	}

	// Proof invariants assume Y coordinate further than X, detect when this is broken
	// so we can do splits to restore the invariant
	bool isExtreme(Eigen::Quaternion<float, 2> orientation, pt2 pos2) {
		auto dvec = orientation._transformVector({ (float) 1.0, 0.0, 0.0 });
		pt2 dir2 = pt2(dvec.x(), dvec.y()).unit();
		pt2 rel = (end - pos2);
		pt2 g = pt2(0, 0) - rel.rebase(dir2); // Translates to vehicle-oriented coordinates
		return fabs(g.x) > (fabs(g.y) + 1.0e-5);
	}

	// Distance of point to the section
	// Tricky geometry because this is distance to *the nearest part of the section*
	double distance(pt2 p) {
		double ret;
		if (isArc) {
			pt2 relM = p - center;
			pt2 relS = start - center;
			pt2 relE = end - center;

			double rAvg = (relE.mag() + relS.mag()) * 0.5;
			double rp = relM.mag();
			return (rp < rAvg) ? rAvg - rp : rp - rAvg;
			//double thM = atan2(relM.y, relM.x), thS = atan2(relS.y, relS.x), thE = atan2(relE.y, relE.x);
			//double thMin = std::min(thS, thE), thMax = std::max(thS, thE);
			//double rMin = rAvg - (rad * 0.5);
			//double rMax = rAvg + (rad * 0.5);
			//double rp = 
			/*if (thM <= thMin) {
				// TODO: this calculation most likely wrong.
				pt2 relMin = pt2(cos(thMin), sin(thMin))*rAvg;
				ret = (relMin - relM).mag();
			}
			else if (thMax <= thM) {
				pt2 relMax = pt2(cos(thMax), sin(thMax))*rAvg;
				ret = (relMax - relM).mag();
			}
			else {
				pt2 rel = pt2(cos(thM), sin(thM))*rAvg;
				ret = (rel - relM).mag();
			}*/
		}
		else {
			pt2 sRelM = p - start;
			pt2 eRelM = p - end;
			pt2 segRel = end - start;
			pt2 mRel = p - start;
			double c = segRel.cos2(mRel);
			double theCos = abs(c);
			pt2 proj = isnan(theCos) ? start : start + (segRel.unit() * theCos * mRel.mag());
			pt2 segRelM = p - proj;
			double a = sRelM.mag(), b = eRelM.mag(), d = segRelM.mag();
			ret = std::min(std::min(a, b), d);
		}
		assert(ret >= 0.0);
		return ret;
	}
	double signedDistance(pt2 p) {
		double d = distance(p);
		if (isLeftOf(p)) {
			return -d;
		}
		else {
			return d;
		}
	}

};

struct Mob { // Mobile entity, a.k.a. the car
	pt2 p; // meters from global Unreal origin
	pt2 v; // meters/sec
	Mob() {};
	Mob(pt2 pp, pt2 vv) : p(pp), v(vv) {};
};

// The whole plan. See .cpp file for docs
class Plan {
public:
	Plan(int nC,
		vector<NodeDatum> nD,
		vector<vector<int>> a,
		Mob v);
	Plan();
	void setMob(double x, double y, double vx, double vy);
	void jumpMob(double x, double y);
	int addNode(NodeDatum nd);
	int addNode(int pred, NodeDatum nd);
	int addNode(vector<int> preds, NodeDatum nd);
	bool isRecurrent();

	bool isAtDeadEnd();
	static const int SUCCESS = 0;
	static const int FAILURE = -1;

	int getCurNode();
	int getNode(int x, NodeDatum& outP);
	int getWaypoint(pt2& outP);
	void lineTo(double rad, double wpX, double wpY, 
		double mX = std::numeric_limits<double>::quiet_NaN(), double mY = std::numeric_limits<double>::quiet_NaN(),
		double vLo = 0.0, double vHi = 0.0);
	void arcTo(double rad, double wpX, double wpY, 
		double cX, double cY,
		double mX = std::numeric_limits<double>::quiet_NaN(), double mY = std::numeric_limits<double>::quiet_NaN(),
		double vLo  = 0.0, double vHi = 0.0, bool isCcw = true);
	int last();
	void connect(int from, int to);
	std::vector<int>& getSuccs(int node);
private:
	int m_nodeCount;
	vector<NodeDatum> m_nodeData;
	vector<vector<int>> m_adj;
	Mob m_vehicle;
	
	int doNode(vector<int> preds, NodeDatum nd);
	bool nodeContains(int i, Mob m);
};