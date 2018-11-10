#pragma once
#include "AirBlueprintLib.h"
#include "common/Geom.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
//#include "CarPawnApi.h"
#include <exception>
#include <set>

extern ENGINE_API float GAverageFPS;

using namespace msr::airlib;


struct NodeDatum {
	pt2 start; // meters from global Unreal origin
	pt2 end;
	pt2 center;
	bool isArc;
	double rad; // metersm
	double vlo; // m/s
	double vhi; // m/s

	double targetVelocity() { return (vlo + vhi) * 0.5; }

	double signedRad() {
		if (isArc) {
			if ((center - start).isLeftOf(end - start)) {
				return rad;
			}
			else {
				return -rad;
			}
		}
		else {
			return std::numeric_limits<double>::infinity();
		}
	}

	bool atEnd(pt2 p) {
		return (p - end).mag() <= rad;
	}

	pt2 tangentAt(pt2 here) {
		if (isArc) {
			//double theMag = (start - center).mag();
			return  (here - center).unit().rot(-0.5*M_PI);

		} else 
			return (start - end).unit();
	}

	pt2 endTangent() {
		pt2 tan =
			  isArc 
			? ((center - end).rot(M_PI * 0.5))
			: start - end;
		return tan.unit();
	}

	pt2 startTangent() {
		pt2 tan =
			isArc
			? ((start - center).rot(M_PI * 0.5))
			: end - start;
		return tan.unit();
	}

	double distance(pt2 p) {
		if (isArc) {
			pt2 relM = p - center;
			pt2 relS = start - center;
			pt2 relE = end - center;

			double thM = atan2(relM.x, relM.y), thS = atan2(relS.x, relS.y), thE = atan2(relE.x, relE.y);
			double thMin = std::min(thS, thE), thMax = std::max(thS, thE);
			double rAvg = (relE.mag() + relS.mag()) * 0.5;
			double rMin = rAvg - (rad * 0.5);
			double rMax = rAvg + (rad * 0.5);
			if (thM <= thMin) {
				pt2 relMin = pt2(cos(thMin), sin(thMin))*rAvg;
				return (relMin - p).mag();
			}
			else if (thMax <= thM) {
				pt2 relMax = pt2(cos(thMax), sin(thMax))*rAvg;
				return (relMax - p).mag();
			}
			else {
				pt2 rel = pt2(cos(thM), sin(thM))*rAvg;
				return (rel - p).mag();
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
				//segRel * ((mRel*segRel) / (segRel*segRel));
			pt2 segRelM = p - proj;
			//) <= rad
			return std::min(std::min(sRelM.mag(), eRelM.mag()), segRelM.mag());
		}
	}
};

struct Mob { // Mobile entity
	pt2 p; // meters from global Unreal origin
	pt2 v; // meters/sec
	Mob() {};
	Mob(pt2 pp, pt2 vv) : p(pp), v(vv) {};
};

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
	static const int PRECISION = 30;
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
		double vLo  = 0.0, double vHi = 0.0);
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