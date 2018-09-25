#pragma once
#include "AirBlueprintLib.h"
#include "common/Geom.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
//#include "CarPawnApi.h"
#include <exception>
#include <set>

extern float GAverageFPS;

using namespace msr::airlib;


struct NodeDatum {
	pt2 p; // meters from global Unreal origin
	double rad; // meters
};

struct Mob { // Mobile entity
	pt2 p; // meters from global Unreal origin
	pt2 v; // meters/sec
	Mob() {};
	Mob(pt2 pp, pt2 vv) : p(pp), v(vv) {};
};

class Plan {
public:
	void setMob(double x, double y, double vx, double vy);
	void jumpMob(double x, double y);
	int addNode(NodeDatum nd);
	int addNode(int pred, NodeDatum nd);
	int addNode(vector<int> preds, NodeDatum nd);
	bool isRecurrent();

	bool isAtDeadEnd();
	static const int PRECISION = 10;
	static const int SUCCESS = 0;
	static const int FAILURE = -1;

	int getCurNode();

	int getWaypoint(pt2& outP);
	void lineTo(double rad, double wpX, double wpY, double mX = std::numeric_limits<double>::quiet_NaN(), double mY = std::numeric_limits<double>::quiet_NaN(), int precision = PRECISION);
private:
	int m_nodeCount;
	vector<NodeDatum> m_nodeData;
	vector<vector<int>> m_adj;
	Mob m_vehicle;
	
	bool nodeContains(int i, Mob m);
	std::vector<int>& getSuccs(int node);
};