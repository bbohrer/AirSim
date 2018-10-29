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
	double rad; // meters

	bool atEnd(pt2 p) {
		return (p - end).mag() <= rad;
	}

	pt2 tangent() {
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
		double mX = std::numeric_limits<double>::quiet_NaN(), double mY = std::numeric_limits<double>::quiet_NaN());
	void arcTo(double rad, double wpX, double wpY, 
		double cX, double cY,
		double mX = std::numeric_limits<double>::quiet_NaN(), double mY = std::numeric_limits<double>::quiet_NaN());
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