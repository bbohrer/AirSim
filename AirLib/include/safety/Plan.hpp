#pragma once
#include "AirBlueprintLib.h"
#include "common/Geom.hpp"
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

	// Feedback controller should aim for middle of speed limits
	double targetVelocity() { return (vlo + vhi) * 0.5; }

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
	bool atEnd(pt2 p) {
		return (p - end).mag() <= rad;
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
			return (start - end).unit();
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

	// Distance of point to the section
	// Tricky geometry because this is distance to *the nearest part of the section*
	double distance(pt2 p) {
		if (isArc) {
			pt2 relM = p - center;
			pt2 relS = start - center;
			pt2 relE = end - center;

			double thM = atan2(relM.y, relM.x), thS = atan2(relS.y, relS.x), thE = atan2(relE.y, relE.x);
			double thMin = std::min(thS, thE), thMax = std::max(thS, thE);
			double rAvg = (relE.mag() + relS.mag()) * 0.5;
			double rMin = rAvg - (rad * 0.5);
			double rMax = rAvg + (rad * 0.5);
			if (thM <= thMin) {
				pt2 relMin = pt2(cos(thMin), sin(thMin))*rAvg;
				return (relMin - relM).mag();
			} else if (thMax <= thM) {
				pt2 relMax = pt2(cos(thMax), sin(thMax))*rAvg;
				return (relMax - relM).mag();
			} else {
				pt2 rel = pt2(cos(thM), sin(thM))*rAvg;
				return (rel - relM).mag();
			}
		}
		else {
			pt2 sRelM = p - start;
			pt2 eRelM = p - end;
			pt2 segRel = end - start;
			pt2 mRel = p - start;
			double c = segRel.cos2(mRel);
			double theCos = abs(c);
			pt2 proj = start + (segRel.unit() * theCos * mRel.mag());
			pt2 segRelM = p - proj;
			return std::min(std::min(sRelM.mag(), eRelM.mag()), segRelM.mag());
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
		double vLo  = 0.0, double vHi = 0.0, bool isCw = true);
	int last();
	void connect(int from, int to);
	std::vector<int>& getSuccs(int node);
private:
	int m_nodeCount;
	vector<NodeDatum> m_nodeData;
	vector<vector<int>> m_adj;
	Mob m_vehicle;
	
	bool nodeContains(int i, Mob m);
};