#include "Experiment.h"
#include "EngineUtils.h"
#include "Geom.hpp"

const int RADIUS_CM = 300;
const int COORDINATES_RELATIVE = 1;
double ACCEL_MAX = 3.5; // m/s^2
double BRAKE_MAX = 1.5; // m/s^2
int CIR_TOL = RADIUS_CM; // cm

using namespace msr::airlib;
// Set to one to record outputs in relative coordinate system 

/* BEGIN IMPLEMENTATION OF ENVIRONMENTS/LEVELS */
// Desired radius of turns

// Construct the "rectangular" level
void AExperiment::loadRect() {
	double off = 12.0;
	double rad = 3.5;
	bool ccw = true;
	double vlo = 0.0;
	double vhi = 18.0;
	//double rad = ((double)RADIUS_CM) / 100.0;
	plan_.lineTo(rad, 0.0, 0.0, 0.00, 30.0 - off, vlo, vhi);
	plan_.arcTo(off, off, 30.0, off, 30.0 - off, 0.0, 30.0 - off, vlo, vhi, ccw);
	plan_.lineTo(rad, off, 30.0, 152.0 - off, 30.0, vlo, vhi);
	plan_.arcTo(off, 152.0, 30.0 - off, 152.0 - off, 30.0 - off, 152.0 - off, 30.0, vlo, vhi, ccw);
	plan_.lineTo(rad, 152.0, 30.0 - off, 152.0, -152.0 + off, vlo, vhi);
	plan_.arcTo(off, 152.0 - off, -152.0, 152.0 - off, -152.0 + off, 152.0, -152.0 + off, vlo, vhi, ccw);
	plan_.lineTo(rad, 152.0 - off, -152.0, off, -152.0, vlo, vhi);
	plan_.arcTo(off, 0.0, -152.0 + off, off, -152.0 + off, off, -152.0, vlo, vhi, ccw);
	plan_.lineTo(rad, 0.0, -152.0 + off, 0.00, 0.0, vlo, vhi);
}

// Helper function to scale plan up/down. Useful when you want to tweak 
// the level's scale because the car needs space to turn
void AExperiment::gridTo(double a, double b, double c, double d, double e, double f, double g) {
	double rate = 0.3;
	plan_.lineTo(a, b * rate, c*rate, d*rate, e*rate, f, g);
}

// Helper function to scale plan up/down. Useful when you want to tweak 
// the level's scale because the car needs space to turn
void AExperiment::aTo(double r, double cornX, double cornY, double fromX, double fromY, double toX, double toY, double f, double g, bool  ccw) {
	double rate = 0.3;
	// aTo(rad, 
	// corn: 0.0,    115.0, 
	// from  0,      -margin, 
	// to:   margin, 0, 
	// vlim: 2.0, 3.0,
	// ccw: b);
	plan_.arcTo(r, cornX*rate + toX, cornY*rate + toY,
		cornX*rate + toX + fromX, cornY*rate + toY + fromY,
		cornX*rate + fromX, cornY*rate + fromY, f, g, ccw);
}

// Construct the "grids" level, which is all right-angle turns, but longer,
// tighter, more complex than the Rectangle level
void AExperiment::loadGrids() {
	double rad = 1.0;//((double)RADIUS_CM) / 100.0;
	double bigrad = 2.5;
	double margin = 4.0; // Turning area
	double bigMargin = 6.0;
	bool b = true;
	double vlo = 0.0;
	double vmid = 11.0;
	double vhi = 18.0;
	gridTo(rad, 0.0, 0.0, 0.0, 115.0 - margin, vlo, vhi); //up
	aTo(rad, 0.0, 115.0, 0, -margin, margin, 0, vlo, vmid, b);
	gridTo(rad, margin, 115.0, 300.0 - margin, 115.0, vlo, vhi); //left
	aTo(rad, 300.0, 115.0, -margin, 0, 0, -margin, vlo, vhi, b);
	gridTo(rad, 300.0, 115.0 - margin, 300.0, margin, vlo, vhi); //down
	aTo(rad, 300.0, 0, 0, margin, -margin, 0, vlo, vhi, b);
	gridTo(rad, 300.0 - margin, 0.0, 200.0 + bigMargin, 0.0, vlo, vhi); //right
	aTo(rad, 200, 0, bigMargin, 0, 0, bigMargin, vlo, vmid, b);
	gridTo(rad, 200.0, bigMargin, 200.0, 115.0 - bigMargin, vlo, vmid); // up
	aTo(bigrad, 200, 115, 0, -bigMargin, -bigMargin, 0, vlo, vmid, !b);
	gridTo(rad, 200.0 - bigMargin, 115.0, 100.0 + margin, 115.0, vlo, vmid); // right
	aTo(rad, 100, 115, margin, 0, 0, -margin, vlo, vhi, !b);
	gridTo(rad, 100.0, 115.0 - margin, 100.0, 0.0 + margin, vlo, vmid); // down
	aTo(rad, 100, 0, 0, margin, margin, 0, vlo, vmid, !b);
	gridTo(rad, 100.0, margin, 200.0 - bigMargin, 0.0, vlo, vmid - 2.0); // left
	aTo(bigrad, 200, 0, -bigMargin, 0, 0, -bigMargin, vlo, vmid - 2.0, b);
	gridTo(bigrad, 200.0, -bigMargin, 200.0, -200.0 + 20 + margin, vlo, vhi);  //bigdown
	gridTo(bigrad, 200.0, -200.0 + 20 + margin, 200.0, -200.0 + bigMargin + 1.0, vlo, vmid - 2.0);  //bigdown
	aTo(bigrad, 200.0, -200.0, 0.0, bigMargin + 1.0, -(bigMargin + 1.0), 0.0, vlo, vmid, b);
	gridTo(rad, 200.0 - (bigMargin + 1.0), -200.0, -90.0, -200.0, vlo, vmid); // right
	gridTo(rad, -90.0, -200.0, -100.0 + bigMargin, -200.0, vlo, vmid - 2.0); // right
	aTo(rad, -100, -200, bigMargin, 0, 0, bigMargin, vlo, vmid, b);
	gridTo(rad, -100.0, -200.0 + bigMargin, -100.0, -100.0 - bigMargin, vlo, vmid); //up
	aTo(rad, -100, -100, 0, -bigMargin, bigMargin, 0, vlo, vhi, b);
	gridTo(rad, -100.0 + bigMargin, -100.0, -bigMargin, -100.0, vlo, vhi); // left
	aTo(rad, 0, -100, -bigMargin, 0, 0, bigMargin, vlo, vhi, !b);
	gridTo(rad, 0.0, -100.0 + bigMargin, 0.0, 0.0, vlo, vhi); //up
}

void AExperiment::parcTo(double rad, pt2 to, pt2 c, pt2 from, bool cW) {
	double OFFX = -200.0;
	double OFFY = -100.0;
	double vlo = 3.0;
	double vhi = 42.0;
	plan_.arcTo(rad, to.x + OFFX, to.y + OFFY, c.x + OFFX, c.y + OFFY, from.x + OFFX, from.y + OFFY, vlo, vhi, cW);
}
// Construct the "4-leaf Clover" level, which is all arcs
void AExperiment::loadClover() {
	double rad = 3.0;//((double)RADIUS_CM) / 100.0;
	//double margin = 3.0; // Turning area
	pt2  ptl = { -200,200 }, pt = { 0,200 }, ptr = { 200,200 },
		pl = { -200,0 }, pr = { 200,0 },
		pbl = { -200,-200 }, pb = { 0,-200 }, pbr = { 200,-200 };
	pt2 pri = { 100, 0 }, pti = { 0,100 }, pli = { -100 ,0 }, pbi = { 0,-100 };
	pt2 ptrb = { 200, 100 }, ptrl = { 100,200 }, ptlr = { -100,200 }, ptlb = { -200,100 },
		pblt = { -200,-100 }, pblr = { -100,-200 }, pbrl = { 100,-200 }, pbrt = { 200,-100 };
	pt2 ptrr = { 300,200 }, ptrt = { 200,300 },
		ptlt = { -200,300 }, ptll = { -300,200 },
		pbll = { -300, -200 }, pblb = { -200,-300 },
		pbrb = { 200,-300 }, pbrr = { 300, -200 };

	parcTo(rad, ptrr, ptr, ptrb, false);
	parcTo(rad, ptrt, ptr, ptrr, false);
	parcTo(rad, ptrl, ptr, ptrt, false);

	parcTo(rad, pti, pt, ptrl, true);
	parcTo(rad, ptlr, pt, pti, true);

	parcTo(rad, ptlt, ptl, ptlr, false);
	parcTo(rad, ptll, ptl, ptlt, false);
	parcTo(rad, ptlb, ptl, ptll, false);

	parcTo(rad, pli, pl, ptlb, true);
	parcTo(rad, pblt, pl, pli, true);

	parcTo(rad, pbll, pbl, pblt, false);
	parcTo(rad, pblb, pbl, pbll, false);
	parcTo(rad, pblr, pbl, pblb, false);

	parcTo(rad, pbi, pb, pblr, true);
	parcTo(rad, pbrl, pb, pbi, true);

	parcTo(rad, pbrb, pbr, pbrl, false);
	parcTo(rad, pbrr, pbr, pbrb, false);
	parcTo(rad, pbrt, pbr, pbrr, false);

	parcTo(rad, pri, pr, pbrt, true);
	parcTo(rad, ptrb, pr, pri, true);
}

void AExperiment::LoadLevel() {
	// Load the selected level
	auto st = vehicle_api_->getCarState();
	//auto pvec = -st.kinematics_estimated.twist.linear;
	auto pvec = st.kinematics_estimated.pose.position;
	pt2 pos2(pvec.x(), pvec.y());

	plan_.jumpMob(pos2.x, pos2.y);
	switch (level_) {
	case LRECT:
		loadRect();
		break;
	case LGRIDS:
		loadGrids();
		break;
	case LCLOVER:
		loadClover();
		break;
	default: break;
	}
	curNode_ = plan_.getCurNode();
	plan_.getNode(curNode_, curND_);
	realCurND_ = curND_;
	// Log the initial constants
	double FPS_est = GAverageFPS;
	double MSPF_est = 1.0 / FPS_est;
	if (!m.isSaved())
		m.saveTo("C:\\Users\\Brandon\\Documents\\out.csv");
	m.consts(MSPF_est, 1.0);
	// Compute and log initial control values
	auto accel = 0; // Not accelerating yet
	// Velocity limits *by time we reach waypoint*
	double vLo = curND_.vlo, vHi = curND_.vhi;
	// *K*urvature
	double k = curND_.isArc ? -1.0 / curND_.signedRad() : 0.0;
	int t = 0; // Time is relative to start of simulation!
	if (COORDINATES_RELATIVE) {
		pt2 rel = (curND_.end - pos2);
		auto orientation = st.kinematics_estimated.pose.orientation;
		auto dvec = orientation._transformVector({ (float) 1.0, 0.0, 0.0 });
		pt2 dir2 = pt2(dvec.x(), dvec.y()).unit();
		//pt2 crdir2 = pt2(-dvec.x(), -dvec.y()).unit();
		lastDir = dir2;
		double altR = (rel.x*rel.x + rel.y*rel.y) / (2.0*rel.x);
		double altK = 1.0 / altR;
		//altK = true ? k : altK;
		if (rel.y <= 0.05) {
			int x = 2;
		}
		m.ctrl(accel, altK, t, vLo, vHi, rel.x, rel.y);
	}
	else {
		m.ctrl(accel, k, t, vLo, vHi, curND_.end.x, curND_.end.y);
	}
	// Currently, all the levels are circular/loopable!
	plan_.connect(plan_.last(), 0);
}

AExperiment::AExperiment(msr::airlib::CarApiBase::CarControls*  keyboard_controls) :
	cruise(false), cruiseDir(false), cruiseDirSet(false),
	lastTime_(FPlatformTime::Seconds()),
	mode_(PLAN),
	fb_(HUMANBOX),
	level_(LRECT),
	curNode_(-1),
	keyboard_controls_(keyboard_controls)
{
}

AExperiment::AExperiment() :
		cruise(false), cruiseDir(false), cruiseDirSet(false),
		lastTime_(FPlatformTime::Seconds()),
		mode_(PLAN),
		fb_(HUMANBOX),
		level_(LRECT),
		curNode_(-1),
		keyboard_controls_(NULL)
{
}

void AExperiment::BeginPlay()
{
	Super::BeginPlay();
	LoadLevel();
}

bool AExperiment::atEnd(pt2 p, pt2 d, double v) {
	auto succs = plan_.getSuccs(curNode_);
	auto next = succs[0];
	NodeDatum nextNode;
	plan_.getNode(next, nextNode);
	double k = 1.0 / nextNode.signedRad();
	double eps = 1.0;
	pt2 rel = nextNode.end - p;
	pt2 g = rel.rebase(d); // Translates to vehicle-oriented coordinates 

	double dev = m.pathDevOf(k, eps, g.x, g.y);
	double devMargin, distMargin;
	switch (level_) {
	case LRECT:
		devMargin = 25.0; distMargin = 0.5;
		break;
	case LGRIDS:
		//devMargin = 82.0;
		//distMargin = 0.5; // for fast pd
		//distMargin = 0.3;
		devMargin = 32.0;
		distMargin = 1.5; // for bb
		break;
	case LCLOVER:
	default:
		//devmargin = 80
		//devmargin 45.0
		devMargin = 20.0; distMargin = 2.0;
		break;
	}
	double theDist = realCurND_.endDist(p, d, v);
	bool b1 = abs(dev) < devMargin;
	bool b2 = theDist < distMargin;
	bool altRet = b1 && b2;
	bool ret = realCurND_.endDist(p, d, v) < 0;
	if (altRet) {
		return true;
	}
	else {
		return false;
	}
}

void AExperiment::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	bool isSafe = true;
	//auto rc_data = getRCData();
	// Get position and velocity of car
	auto st = vehicle_api_->getCarState();
	auto pvec = //-st.kinematics_estimated.twist.linear;
		st.kinematics_estimated.pose.position;
	auto orientation = st.kinematics_estimated.pose.orientation;
	auto vvec = orientation._transformVector({ (float)st.speed,0.0f,0.0f });
	auto dogvec = orientation._transformVector({ (float) 1.0, 0.0, 0.0 });
	auto logvec = orientation._transformVector({ (float) 0.0, 1.0, 0.0 });
	//vvec;
	pt2 dir2 = pt2(dogvec.x(), dogvec.y()).unit();
	pt2 ldir2 = pt2(logvec.x(), logvec.y()).unit();
	pt2 pos2(pvec.x(), pvec.y());
	auto speed = st.speed;
	auto vx = vvec[0], vy = vvec[1], vz = vvec[2];

	// time delta
	double newTime = FPlatformTime::Seconds();
	// TODO:
	//double ep = (newTime - lastTime_);
	double ep = 0.10;
	lastTime_ = newTime;
	pt2 way = curND_.end;

	switch (mode_) {
		// Manual keyboard-based driving. Irrelevant for autonomous driving,
		// Useful for developing new environments / playing around
	case KEYBOARD: {
		// Notify planner of new position
		plan_.setMob(pvec[0], pvec[1], vvec[0], vvec[1]);
		// Ask planner for current or next waypoint
		if (-1 == curNode_) {
			curNode_ = plan_.getCurNode();
		}
		else if (atEnd(pos2, dir2, speed)) {
			auto succs = plan_.getSuccs(curNode_);
			curNode_ = succs[0];
			if (curNode_ == 0)
				m._finished = true;
		}
		plan_.getNode(curNode_, curND_);
		realCurND_ = curND_;
		/*while (curND_.isExtreme(orientation,pos2)) {
			curND_ = curND_.first();
		}*/
		// Print angular velocity ON-SCREEN for useful debugging
		char buf[256];
		snprintf(buf, 256, "%f", st.kinematics_estimated.twist.angular.z());

		// Main case of keyboard control
		if (mode_ == KEYBOARD && NULL != keyboard_controls_) {
			current_controls_ = *keyboard_controls_;
		}
		else {
			double aUp = current_controls_.throttle * ACCEL_MAX;
			double aDown = current_controls_.brake * -BRAKE_MAX;
			double a = aUp + aDown;
			double k = curND_.isArc ? -1.0 / curND_.signedRad() : 0.0;
			double vLo = curND_.vlo, vHi = curND_.vhi;
			if (COORDINATES_RELATIVE) {
				auto dvec = orientation._transformVector({ (float) 0.0, 1.0, 0.0 });
				pt2 dir2 = pt2(dvec.x(), dvec.y()).unit();
				pt2 rel = (way - pos2);
				pt2 g = pt2(0, 0) - rel.rebase(dir2); // Translates to vehicle-oriented coordinates
				pt2 gOld = pt2(0, 0) - rel.rebase(lastDir);
				m.sense(ep, speed, gOld.x, gOld.y);
				if (gOld.y <= 0.05) {
					int x = 2;
				}

				m.afterSense();
				if (g.y <= 0.05) {
					int x = 2;
				}
				m.ctrl(a, k, 0, vLo, vHi, g.x, g.y);
				snprintf(buf, 256, "%f %f", m.pathDevOf(k, 1.0, g.x, g.y), curND_.endDist(pos2, dir2, speed));
				snprintf(buf, 256, "%d\t%d\t%d\t%d", m.b1, m.b2, m.b3, m.b4);

				if (m._extCtrlMon || !m.ctrlOk()) {
					isSafe = false;
					std::string msg;
					m.ctrlErr(msg);
					current_controls_.throttle = 0;
					current_controls_.brake = 1;
				}
				m.afterCtrl();
			}
			else {
				pt2 g = way;
				m.sense(ep, speed, g.x, g.y);
				m.afterSense();
				m.ctrl(a, k, 0, vLo, vHi, g.x, g.y);
				m.afterCtrl();
			}
			lastDir = ldir2;
			break;
		}
		// Autonomous controller that follows the plan!!
	case PLAN: {
		// Notify planner of new position
		plan_.setMob(pvec[0], pvec[1], vvec[0], vvec[1]);
		// Ask planner for current or next waypoint
		if (-1 == curNode_) {
			curNode_ = plan_.getCurNode();
			//} else if (curND_.atEnd(pos2)) {
		}
		else if (atEnd(pos2, dir2, speed)) {
			auto succs = plan_.getSuccs(curNode_);
			cruise = false;
			cruiseDir = false;
			cruiseDirSet = false;
			curNode_ = succs[0];
			if (curNode_ == 0)
				m._finished = true;
		}
		plan_.getNode(curNode_, curND_);
		realCurND_ = curND_;
		int whr;
		while (0 != (whr = curND_.isExtreme(orientation, pos2/*, speed*/))) {
			if (whr == 1) {
				curND_ = curND_.first();
			}
			else {
				curND_ = curND_.second();
			}
		}
		pt2 way = curND_.end;
		pt2 wayDiff = curND_.tangentAt(pos2, curND_.isCcw);
		//pt2 wayDiff = (way - pos2).unit(); // relative
		// Distance until waypoint ENTERED
		// TODO: Consider using NodeDatum.distance() function here
		double distBuffer = level_ == LCLOVER ? 2.5 : 0.0;
		auto dist = (pt2(pvec[0], pvec[1]) - way).mag() - distBuffer;
		// Velocity after one timestep acceleration, RELATIVE to target vel
		double lerp = (fb_ == PDOVERDRIVE ? 2.0 : fb_ == PDFAST ? 0.8 : fb_ == PDSLOW ? 0.3 : 0.35);
		auto vv = (st.speed + ACCEL_MAX * ep) - curND_.targetVelocity(lerp);
		// Distance after one timestep
		auto dd = fmax(0.0, dist - ((CIR_TOL / 100.0) + ep * st.speed + ep * ep * 0.5f * ACCEL_MAX));
		// Do we need to start braking?
		// TODO: Braking is currently bang-bang, would be great to add PD
		double HARD_LIMIT = 28.0;
		bool close;
		// Heuristic: Since the monitoring conditions are more conservative
		// on curves than straightaways, immediately observe the speed limit
		// on turns, but observe it late-as-safely-possible on straight
		if (curND_.isArc) {
			close = vv >= 0 && st.speed + ACCEL_MAX * ep >= curND_.vhi;
		}
		else {
			close = vv >= 0 && (((vv * vv >= dd / (2.0f * BRAKE_MAX))) || st.speed > HARD_LIMIT);
		}
		ctrlTicks++;
		if (ctrlTicks % 15 == 0) {
			slowClose = close;
		}
		if (!cruiseDirSet) {
			cruiseDir = close;
			cruiseDirSet = true;
		}
		else {
			if (!cruise && (close != cruiseDir)) {
				cruise = true;
			}
		}
		if (cruise) close = slowClose;
		if (fb_ == PDOVERDRIVE) close = false;
		// For bang-bang steering
		bool goLeft = wayDiff.isLeftOf(dir2);

		// Data structure for storing/reporting control decision
		msr::airlib::CarApiBase::CarControls ai_controls = {};
		ai_controls.brake = 0.0f;
		ai_controls.gear_immediate = true;
		ai_controls.manual_gear = 0;
		ai_controls.is_manual_gear = false;

		// Which control mode? Main control computation here
		switch (fb_) {
		case PDNARROW:
		case PDWIDE:
		case PDOVERDRIVE:
		case PDFAST:
		case PDSLOW:
		case HUMANBOX:
		case PDNORMAL: {
			char buf[256];
			// PD controller uses distance-to-path as proportional
			// and orientation as "derivative" to control steering
			//double dist = curND_.signedDistance(pos2);
			double magP = wayDiff.sin2(dir2);// .sin2(doubleUnit);
			auto cos = wayDiff.cos2(dir2);
			auto sq = sqrt(1 - cos * cos);
			bool l = wayDiff.isLeftOf(dir2);

			//float* leftsD = st.kinematics_estimated.twist.angular.data();
			pt2 theRel = pos2 - curND_.start;
			pt2 tanAt = curND_.tangentAt(pos2, curND_.isCcw);
			//bool signage = theRel.isLeftOf(tanAt);
			double leftP = magP; //signage ? -magP : magP;
			double leftD = curND_.signedDistance(pos2);;

			snprintf(buf, 256, "%f", leftD);
			double P, D, Pfac, Dfac;
			switch (level_) {
			case LGRIDS:
				P = 6.0; D = 0.75;
				break;
			case LCLOVER:
				P = 6.0; D = 0.75;
				break;
			case LRECT:
			default:
				P = 2.0; D = 0.25;
			}
			switch (fb_) {
			case PDNARROW: Pfac = 1.5; Dfac = 1.5; break;
			case PDWIDE: Pfac = 0.7; Dfac = 0.7; break;
			default: Pfac = 1.0; Dfac = 1.0; break;
			}
			double steer = Pfac * P * leftP + Dfac * D * leftD;
			if (steer >= 1 || steer <= -1) {
				int x = 1 + 1;
			}
			if (fb_ == HUMANBOX && NULL != keyboard_controls_) {
				ai_controls = *keyboard_controls_;
			}
			else if (cruise) {
				ai_controls.steering = steer;
				// braking still bangbang
				ai_controls.throttle = close ? 0.0f : 0.99f;
				ai_controls.brake = close ? 1.0f : 0.0f;
			}
			else {
				ai_controls.steering = steer;
				// braking still bangbang
				ai_controls.throttle = close ? 0.0f : 0.99f;
				ai_controls.brake = close ? 1.0f : 0.0f;
			}
			//On-screen debugging
			snprintf(buf, 256, "%2.2f*%2.2f=%2.2f,\t%2.2f*%2.2f=%2.2f,\tsum=%2.2f", P, leftP, P*leftP, D, leftD, D*leftD, steer);
			break;
		}
		case BANGBANG: { // bang-bang control of both steering and braking
			const double MAX_STEER = 1.0;
			// 0.9, 0.5: grid
			ai_controls.steering = 0.3 * (goLeft ? -MAX_STEER : MAX_STEER);
			ai_controls.throttle = 1.0 * (close ? 0.0f : 0.99f);
			break;
		}
		}

		current_controls_ = ai_controls;

		// Logging code
		double vxy = vvec[0] * way.y;
		double xvy = way.x * vvec[1];
		double aUp = ai_controls.throttle * ACCEL_MAX;
		double aDown = ai_controls.brake * -BRAKE_MAX;
		double a = aUp + aDown;
		double k = curND_.isArc ? -1.0 / curND_.signedRad() : 0.0;
		double vLo = curND_.vlo, vHi = curND_.vhi;
		char buf[256];
		if (COORDINATES_RELATIVE) {
			auto dvec = orientation._transformVector({ (float) 1.0, 0.0, 0.0 });
			pt2 dir2 = pt2(dvec.x(), dvec.y()).unit();
			pt2 rel = (way - pos2);
			pt2 gOld = rel.rebase(lastDir);
			pt2 g = rel.rebase(dir2); // Translates to vehicle-oriented coordinates
			/* *K*urvature */
			double STOPX = 0.05;
			double altK;
			if (fabs(g.x) <= STOPX) {
				altK = 0;
				g.x = 0;
			}
			else {
				altK = (2.0*g.x) / (g.x*g.x + g.y*g.y);
			}
			m.sense(ep, speed, g.x, g.y);
			if (m._extPlantMon) {
				isSafe = false;
			}
			m.afterSense();

			//altK = true ? k : altK;
			m.ctrl(a, altK, 0, vLo, vHi, g.x, g.y);
			snprintf(buf, 256, "%f", a);
			if (m._extCtrlMon || !m.ctrlOk()) {
				isSafe = false;
				m.ctrlFallback(-BRAKE_MAX);
				std::string msg;
				m.ctrlErr(msg);

				current_controls_.throttle = 0;
				current_controls_.brake = 1;
			}
			m.afterCtrl();
			snprintf(buf, 256, "%f %%", 100.0 * m.ctrlFailRate());
			snprintf(buf, 256, "%f %%", 100.0 * m.caseFailRate());
			snprintf(buf, 256, "%f %f", m.pathDevOf(k, 1.0, g.x, g.y), curND_.endDist(pos2, dir2, speed));
			snprintf(buf, 256, "%d\t%d\t%d\t%d", m.b1, m.b2, m.b3, m.b4);


			if (true || m._finished) {
				snprintf(buf, 256, "%f", (double)m.ctrl_fails / (double)m.ctrl_ticks);
				//UAirBlueprintLib::LogMessageString("CTRL: ", buf, LogDebugLevel::Informational);
				snprintf(buf, 256, "%f", (double)m.phys_fails / (double)m.phys_ticks);
				//UAirBlueprintLib::LogMessageString("PLANT: ", buf, LogDebugLevel::Informational);
				snprintf(buf, 256, "%f", (double)m._velAvg);
				//UAirBlueprintLib::LogMessageString("VEL: ", buf, LogDebugLevel::Informational);
			}

			m.afterCtrl();
		}
		else {
			pt2 g = way;
			m.sense(ep, speed, g.x, g.y);
			m.ctrl(a, k, 0, vLo, vHi, g.x, g.y);
		}
		lastDir = dir2;

		// On-screen log
		snprintf(buf, 256, "%d: (%f, %f)", vvec[0] >= 0, vxy, xvy);
		snprintf(buf, 256, "(%f, %f)", pvec[0], pvec[1]);
		snprintf(buf, 256, "(%f, %f)", vvec[0], vvec[1]);
		snprintf(buf, 256, "from %d(%f,%f) to (%f,%f), left:%d", curNode_, curND_.start.x, curND_.start.y, way.x, way.y, goLeft);
		snprintf(buf, 256, "@(%f,%f)  ->(%f,%f)", pos2.x, pos2.y, dir2.x, dir2.y);

		snprintf(buf, 256, "%d (%f)", curNode_, curND_.endDist(pos2, dir2, speed));
		//TODO linker error: UAirBlueprintLib::LogMessageString("Nav: ", buf, LogDebugLevel::Informational);

		current_controls_ = ai_controls;

		/*	case  SPIN: { // Test mode: Spin in circles, see how fast accel+brake on curve are
				CarPawnApi::CarControls ai_controls = {};
				ai_controls.brake = 0.0f;
				ai_controls.steering = -30.0f;
				ai_controls.gear_immediate = true;
				ai_controls.throttle = 0.99f;
				current_controls_ = ai_controls;
				if (!vehicle_api_->isApiControlEnabled()) {
					//all car controls from anywhere must be routed through API component
					vehicle_api_->setCarControls(current_controls_);
				}
				else {
					current_controls_ = vehicle_api_->getCarControls();
				}

				break;
			}
			default:
				break;
			}*/
			//char buf[256];
		static double lastBound = 1.0;
		if (m._finished) {
			snprintf(buf, 256, "%f", (double)m.ctrl_fails / (double)m.ctrl_ticks);
			//TODO linker error: UAirBlueprintLib::LogMessageString("CTRL: ", buf, LogDebugLevel::Informational);
			snprintf(buf, 256, "%f", (double)m.phys_fails / (double)m.phys_ticks);
			//TODO linker error: UAirBlueprintLib::LogMessageString("PLANT: ", buf, LogDebugLevel::Informational);
			snprintf(buf, 256, "%f", (double)m._velAvg);
			//TODO linker error: UAirBlueprintLib::LogMessageString("VEL: ", buf, LogDebugLevel::Informational);
			snprintf(buf, 256, "%f", (double)m.boundaryDist());
			//TODO linker error: UAirBlueprintLib::LogMessageString("BOUND: ", buf, LogDebugLevel::Informational);
		}

		const bool QUANT = false;
		vehicle_api_->setCarControls(current_controls_);
		//vehicle_api_->getPawn()->updateMaterial(QUANT ? m.boundaryDist() : (isSafe ? 300000000.0 : -100000000.0));
	}
	}
	}
}