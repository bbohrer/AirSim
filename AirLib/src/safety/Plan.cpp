/* 
 * The Plan represents all the places that the vehicle wishes to go in the future.
 * It is represented as a directed graph where the vertices are path sections (lines or arcs).
 * Edges determine which path sections may succeed each other.
 *
 * Different graph structures represent different styles of driving.
 * A chain graph represents a deterministic plan, where the only flexibility provided to the controller is which part of
 * the waypoint circle it drives through (this certain level of control is necessary to account for sensor/actuator imperfection and for imperfect computations in
 * monitors). 
 * Any other DAG with a common sink represents a "go-to" style mission with added robustness: the controller is free to pick between multiple paths, say
 * when one of them is determined to be blocked or dangerous
 * Any other DAG with multiple sinks represents a "go to any of these places" style mission, arguably the least useful.
 * Cyclic graphs are quite useful as they represent continuous patrol missions. As with go-to missions, they can have branches
 * to provide robustness.
 *
 * See .hpp file for data structure details and helpers
 */

#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "safety/Plan.hpp"
#include "safety/CubeActor.h"
#include "common/Geom.hpp"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include <exception>
#include <set>

using namespace msr::airlib;

// constructor
Plan::Plan(int nC,
	vector<NodeDatum> nD,
	vector<vector<int>> a,
	Mob v) : m_nodeCount(nC), m_nodeData(nD), m_adj(a), m_vehicle(v) {}
Plan::Plan() : m_nodeCount(0), m_nodeData(), m_adj(), m_vehicle() {}

	void Plan::setMob(double x, double y, double vx, double vy) {
		m_vehicle = Mob(pt2(x,y),pt2(vx,vy));
	}
	void Plan::jumpMob(double x, double y) {
		m_vehicle = Mob(pt2(x,y),pt2(0,0));
	}

	// Add node (path section), optionally with predecessors
	int Plan::addNode(NodeDatum nd) { return addNode(vector<int>(),nd); }
	int Plan::addNode(int pred, NodeDatum nd) { 
		vector<int> preds;
		preds.push_back(pred);
		int ret = addNode(preds, nd);
		bool RENDER_GEOM = false;
		// Debugging feature: Draw 3D geometry on screen corresponding to the plan
		if (RENDER_GEOM) {
			UWorld* uw = GWorld;
			FVector location(0.0, 0.0, 0.0);
			auto rot = FRotator(0, 0, 0);
			auto cls = ACubeActor::StaticClass();
			FActorSpawnParameters fasp;
			ESpawnActorCollisionHandlingMethod sachm = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			fasp.SpawnCollisionHandlingOverride = sachm;
			ACubeActor* act = uw->SpawnActor<ACubeActor>(cls, location, rot, fasp);
			act->xCm = nd.center.x * 100;
			act->yCm = nd.center.y * 100;
			act->zCm = 500 + (nd.rad * 100);
			act->radiusCm = (int)(nd.rad * 100.0);
			act->CreateCube();
			act->GetRootComponent()->DestroyPhysicsState();
		}
		return ret;
	}

	int Plan::doNode(vector<int> preds, NodeDatum nd) {
		int thisNode = m_nodeCount++;
		for (auto it = preds.begin(); it != preds.end(); it++) {
			int i = *it;
			m_adj[i].push_back(thisNode);
		}
		m_nodeData.push_back(nd);
		m_adj.push_back(vector<int>());
		return thisNode;
	}

	// Update graph data structures for added node
	int Plan::addNode(vector<int> preds, NodeDatum nd) {
		std::vector<NodeDatum> nds = { nd };
		for (auto it = nds.begin(); it != nds.end();) {
			if (it->length() >= it->splitSize()) {
				NodeDatum prev = *it;
				NodeDatum fst = prev.first();
				NodeDatum snd = prev.second();
				it = nds.insert(it, fst);
				(*(it+1)) = snd;
			} else {
				it++;
			}	
		}
		int j;
		for (int i = 0; i < nds.size(); i++) {
			j = doNode(preds, nds[i]);
			preds = { j };
		}
		return j;
	}
	
	// Meaning that we will never run out of places to go regardless of
	// which branches we take
	bool Plan::isRecurrent() {
		for (auto it = m_adj.begin(); it != m_adj.end(); it++) {
			if (it->size() == 0) {
				return false;
			}
		}
		return true;
	}

	// Returns true if no way out of current waypoint
	bool Plan::isAtDeadEnd() {
		std::set<int> nodesAt;
		for (int i = 0; i < m_nodeCount; i++) {
			if (nodeContains(i, m_vehicle)) {
				nodesAt.insert(i);
			}
		}
		// No out-edges, except into the current set
		for (auto it = nodesAt.begin(); it != nodesAt.end(); it++) {
			for (auto succ = m_adj[*it].begin(); succ != m_adj[*it].end(); succ++) {
				// One of the waypoints we're in offers a way forward, not stuck here
				if (nodesAt.find(*succ) == nodesAt.end()) {
					return false;
				}
			}
		}

		// Recurrent (sub) sets are never dead end.
		for (auto it = nodesAt.begin(); it != nodesAt.end(); it++) {
			bool itHasSucc = false;
			for (auto succ = m_adj[*it].begin(); succ != m_adj[*it].end(); succ++) {
				if (nodesAt.find(*succ) != nodesAt.end()) {
					itHasSucc = true;
					break;
				}
			}
			// One of the nodes we're in doesn't offer a way into any of its neighbors,
			// so it's not a recurrent (sub) set and thus a dead end
			if (!itHasSucc) {
				return true;
			}
		}
		// if we reach this point, must have been a recurrent set, not dead-end
		return false; 
	}
	static const int SUCCESS = 0;
	
	std::vector<int>& Plan::getSuccs(int node) { return m_adj[node]; }

	// get node by index
	int Plan::getNode(int x, NodeDatum& outP) {
		if (x >= m_nodeCount) {
			return FAILURE;
		}
		outP = m_nodeData[x];
		return SUCCESS;
	};
	// Get "closest", "next" node heuristically from vehicle position and orientation
	int Plan::getWaypoint(pt2& outP) {
		auto curr = getCurNode();
		if (curr < 0) {
			return FAILURE;
		}
		auto currData = m_nodeData[curr];
		Mob m = m_vehicle;
		pt2 v = m.v;

		// "Metric" tells us how "close" / "preferable" a waypoint is by combination of distance
		// and difference-in-orientation
		double minMetric = std::numeric_limits<double>::infinity();
		double minSin = 0.0;
		pt2 minPoint = {0,0};
		auto succs = getSuccs(curr);
		if (succs.empty()) {
			return FAILURE;
		}
		pt2 mc = m.p;
		for (auto it = succs.begin(); it != succs.end(); it++) {
			int i = *it;
			auto node = m_nodeData[i];
			double dist = node.distance(mc);
			double theSin = node.startTangent().sin2(v);
			// Metric is sin-of-angle-between * distance
			auto metr = theSin * dist;
			if (metr < minMetric) {
				minMetric = metr;
				minSin = theSin;
				minPoint = node.end;
			}
		}
		outP = minPoint;
		return SUCCESS;
	}

	/* Returns index of the node containing the vehicle. 
	 * If mob is in multiple nodes, returns the one whose center is closest.
	 * If in is in no nodes, returns negative int. */
	int Plan::getCurNode() {
		auto mob = m_vehicle;
		vector<int> curNodes;
		for (int i = 0; i < m_nodeCount; i++) {
			if (nodeContains(i, mob) || true) {
				curNodes.push_back(i);
			}
		}
		if (curNodes.size() == 0) {
			return -1;
		}

		double minDistSq = std::numeric_limits<double>::infinity();
		int iMin = -1;
		for (auto it = curNodes.begin(); it != curNodes.end(); it++) {
			NodeDatum dat = m_nodeData[*it];
			auto c = dat.center;
			auto distSq = (c.x - mob.p.x)*(c.x - mob.p.x) + (c.y - mob.p.y)*(c.y - mob.p.y);
			if (distSq < minDistSq) {
				minDistSq = distSq;
				iMin = *it;
			}
		}

		return iMin;
	}

	void Plan::lineTo(double rad, double mX, double mY, double wpX, double wpY, double vLo, double vHi) {
		// NaN used as sentinel value to represent default behavior, which is to take mob position from the plan's
		// idea of where the vehicle is.
		if (std::isnan(mX) || std::isnan(mY)) {
			mX = m_vehicle.p.x; mY = m_vehicle.p.y;
		}
		int curNode = m_nodeCount-1;
		NodeDatum nd = { {mX, mY}, {wpX,wpY,}, {}, false, rad, vLo, vHi };
		if (curNode < 0) {
			curNode = addNode(nd);
		} else {
			curNode = addNode(curNode, nd);
		}
	}

	// Create a new arc-shaped section
	void Plan::arcTo(double rad, double wpX, double wpY, double cX, double cY, double mX, double mY, double vLo, double vHi, bool ccW) {
		NodeDatum nd = { {mX, mY}, {wpX,wpY,}, {cX,cY}, true, rad, vLo, vHi, ccW };
		if (std::isnan(mX) || std::isnan(mY)) {
			mX = m_vehicle.p.x; mY = m_vehicle.p.y;
		}
		int curNode = m_nodeCount - 1;
		if (curNode < 0) {
			curNode = addNode(nd);
		} else {
			curNode = addNode(curNode, nd);
		}
	}

	// Does the node contain the vehcile? 
	// Basically nasty geometry
	bool Plan::nodeContains(int i, Mob m) {
		pt2 mc = m.p;
		NodeDatum dat = m_nodeData[i];
		if (dat.isArc) {
			pt2 relM = mc - dat.center;
			pt2 relS = dat.start - dat.center;
			pt2 relE = dat.end - dat.center;
			double thM = atan2(relM.x, relM.y), thS = atan2(relS.x, relS.y), thE = atan2(relE.x, relE.y);
			bool inTh = std::min(thS, thE) <= thM && thM <= std::max(thS, thE);
			if (!inTh) {
				return false;
			}
			double rAvg = (relE.mag() + relS.mag()) * 0.5;
			double rMin = rAvg - (dat.rad * 0.5);
			double rMax = rAvg + (dat.rad * 0.5);
			double rm = relM.mag();
			return rMin <= rm && rm <= rMax;
		} else {
			pt2 sRelM = mc - dat.start;
			pt2 eRelM = mc - dat.end;
			pt2 segRel = dat.end - dat.start;
			pt2 mRel = mc - dat.start;
			pt2 proj = segRel*((mRel*segRel) / (segRel*segRel));
			pt2 segRelM = mc - proj;
			return std::min(std::min(sRelM.mag(), eRelM.mag()), segRelM.mag()) <= dat.rad;
		}
	}

	int Plan::last() { return m_nodeCount - 1; }
	void Plan::connect(int from, int to) {
		m_adj[from].push_back(to);
	}