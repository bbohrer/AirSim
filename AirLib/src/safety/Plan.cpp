/* 
 * The Plan represents all the places that the vehicle wishes to go in the future.
 * It is represented as a directed graphs where the vertices are waypoints with radii.
 * The radius determines the max safe velocity within that waypoint.
 * The edges are the intersections between adjacent waypoint circles; their maximum safe velocity is determined 
 * by the radius of the largest inscribed circle of the this intersection.
 * (this relationship stems from the fact that braking and steering are used for safety and that braking and steering effectiveness
 *  depend on velocity)
 * Generally speaking, a larger number of circles means that these intersections are also larger so the speed limits are higher.
 * However, a large number of circles also demands a faster control cycle so that we do not, say, pass through 3 circles in the plan
 * by the next time the controller loops.
 * One of our goals in configuration is to set the fidelty on the plan so that it exploits available processor speed without surpassing it.
 * The update rate of sensors is also relevant, but in cases where the processor is significantly faster than the sensors,
 * estimation can be used to run the controller at higher rate
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
 * TODO (decreasing priority):
 *  Basic structure for storing plans
 *  Derived computations: speed limits, connectivity, cyclicity
 *  High-level utilities (draw line, draw arc, draw splin)
 *  Connect to frame rate
 *  Build appropriate environment for this
 *  Build controller around this
 *  Feasability /robustness analysis
 *  Estimators
 *  Frequency-of-visit data for better nondeterministic patrols
 */

//#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "safety/Plan.hpp"
#include "safety/CubeActor.h"
#include "common/Geom.hpp"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include <exception>
#include <set>

//#include "ProceduralMeshComponent.h"
//#include "ActorFactories/ActorFactoryBasicShape.h"
//#include "ActorFactoryBasicShape.generated.h"
//#include "AssetRegistryModule.h"
//#include "AssetData.h"

//#include "CarPawnApi.h"

using namespace msr::airlib;


/*struct NodeDatum {
	double x;   // meters from global Unreal origin
	double y;   // meters from global Unreal origin
	double rad; // meters
};

struct Mob { // Mobile entity
	double x;
	double y;
	double vx;
	double vy;
};
*/
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

	int Plan::addNode(NodeDatum nd) { return addNode(vector<int>(),nd); }
	static int gFoo = 0;
	int Plan::addNode(int pred, NodeDatum nd) { 
		auto templ = NULL;
		auto owner = NULL;
		FVector pos = {}; 
		
		UWorld* uw = GWorld;
		//AActor* ac = SpawnActor<CubeActor>();
		FVector location( 0.0, 0.0, 0.0);
		gFoo++;
		auto rot = FRotator(0, 0, 0);
		auto cls = ACubeActor::StaticClass();
		// FActorSpawnParameters
		FActorSpawnParameters fasp; 
		ESpawnActorCollisionHandlingMethod sachm = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		fasp.SpawnCollisionHandlingOverride = sachm;
		ACubeActor* act = uw->SpawnActor<ACubeActor>(cls, location, rot, fasp);
		act->xCm = nd.p.x * 100;
		act->yCm = nd.p.y * 100;
		act->zCm = 500 + (nd.rad * 100);
		act->radiusCm = (int)(nd.rad * 100.0);
		act->CreateCube();
		act->GetRootComponent()->DestroyPhysicsState();
		char buf[256];
		snprintf(buf, 256, "(%d, %d, %d)", act->xCm, act->yCm, act->zCm);
		UAirBlueprintLib::LogMessageString("CUBE: ", buf, LogDebugLevel::Informational);

		int x = 2 + 2;
		//CubeActor ca(100);
//		UWorld::SpawnActor()
		//UActor *actor();
		
		//FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
		//IAssetRegistry& AssetRegistry = AssetRegistryModule.Get();
//
	//	auto ad = AssetRegistry.GetAssetByObjectPath(UActorFactoryBasicShape::BasicCube);
		/** Initialize NewActorClass if necessary, and return default actor for that class. */
		//UActorFactoryBasicShape ufac;
		//AActor* foo = ufac.GetDefaultActor(ad);

		/** Initialize NewActorClass if necessary, and return that class. */
		//UClass* uc = ufac.GetDefaultActorClass(ad);

		/** Given an instance of an actor pertaining to this factory, find t *
		AActor a =
			UWorld::SpawnActor(uclass, NULL, &pos, NULL, NULL, true, false, owner, NULL, true);
			SpawnActor();
		UPrimitiveComponent cmp;
		a.Destroy();*/
		vector<int> preds;
		preds.push_back(pred);
		return addNode(preds,nd); 
	}
	int Plan::addNode(vector<int> preds, NodeDatum nd) {
		int thisNode = m_nodeCount++;
		for (auto it = preds.begin(); it != preds.end(); it++) {
			int i = *it;
			m_adj[i].push_back(thisNode);
		}
		m_nodeData.push_back(nd);
		m_adj.push_back(vector<int>());
		return thisNode;
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
	static const int PRECISION = 30;	
	static const int SUCCESS = 0;
	
	std::vector<int>& Plan::getSuccs(int node) { return m_adj[node]; }

	int Plan::getNode(int x, NodeDatum& outP) {
		if (x >= m_nodeCount) {
			return FAILURE;
		}
		outP = m_nodeData[x];
		return SUCCESS;
	};
	int Plan::getWaypoint(pt2& outP) {
		auto curr = getCurNode();
		if (curr < 0) {
			return FAILURE;
		}
		auto currData = m_nodeData[curr];
		Mob m = m_vehicle;
		pt2 v = m.v;
		double minMetric = std::numeric_limits<double>::infinity();
		double minSin = 0.0;
		pt2 minPoint = {0,0};
		auto succs = getSuccs(curr);
		if (succs.empty()) {
			return FAILURE;
		}
		for (auto it = succs.begin(); it != succs.end(); it++) {
			int i = *it;
			auto node = m_nodeData[i];
			auto nRel = node.p - m.p;
			auto sin = nRel.sin2(v);
			auto dist = nRel.mag();
			auto metr = sin * dist;
			if (metr < minMetric) {
				minMetric = metr;
				minSin = sin;
				minPoint = node.p;
			}
		}
		outP = minPoint;
		return SUCCESS;
	}

	/* Returns index of the node containing the mob. 
	 * If mob is in multiple nodes, returns the one whose center is closest.
	 * If in is in no nodes, returns negative int. */
	int Plan::getCurNode() {
		auto mob = m_vehicle;
		vector<int> curNodes;
		for (int i = 0; i < m_nodeCount; i++) {
			if (nodeContains(i, mob)) {
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
			auto c = dat.p;
			auto distSq = (c.x - mob.p.x)*(c.x - mob.p.x) + (c.y - mob.p.y)*(c.y - mob.p.y);
			if (distSq < minDistSq) {
				minDistSq = distSq;
				iMin = *it;
			}
		}

		return iMin;
	}

	void Plan::lineTo(double rad, double wpX, double wpY, double mX, double mY, int precision) {
		// NaN used as sentinel value to represent default behavior, which is to take mob position from the plan's
		// idea of where the vehicle is.
		if (std::isnan(mX) || std::isnan(mY)) {
			mX = m_vehicle.p.x; mY = m_vehicle.p.y;
		}
		if (wpX == mX || wpY == mY) {
			return;
		}
		auto deltaX = (wpX - mX) / precision;
		auto deltaY = (wpY - mY) / precision;
		int curNode = 0;
		if ((curNode = getCurNode()) < 0) {
			return;
		}
		int i = 0;
		do {
			mX += deltaX; mY += deltaY;
			NodeDatum curDatum = { {mX, mY}, rad };
			curNode = addNode(curNode, curDatum);
		} while (++i < PRECISION);
	}

	bool Plan::nodeContains(int i, Mob m) {
		NodeDatum dat = m_nodeData[i];
		auto cx = dat.p.x, cy = dat.p.y;
		auto distSq = (cx - m.p.x)*(cx - m.p.x) + (cy - m.p.y)*(cy - m.p.y);
		return dat.rad*dat.rad >= distSq;
	}