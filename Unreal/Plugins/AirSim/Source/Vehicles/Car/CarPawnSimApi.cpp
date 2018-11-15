/* 
 * This file is the "main" implementation for the robot logging and monitoring.
 * It also contains some test code that can be used to figure out acceleration/braking
*/

#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "CarPawnApi.h"
#include <exception>


/* BEGIN LOGGING CODE */
using namespace msr::airlib;
// The file that the CSV log will be written to. See printConsts()
FILE* fd = NULL;

// Set to one to record outputs in relative coordinate system 
const int COORDINATES_RELATIVE = 1;

/* Prints the system constants to CSV log.
 * Also opens the log if it hasn't been opened yet.
 * The log gets stored in a hard-coded path below, change it as suitable.
 * If opening the log fails, writes to stdout */
void printConsts(int T, int eps) {
	if (!fd) {
		fd = fopen("C:\\Users\\Brandon\\Documents\\out.csv", "w");
		if (!fd) fd = stdout;
	}
	// Write header
	fprintf(fd, "t(T),v(eps),xg,yg,a,k,t,vh,vl,xg,yg\n");
	// Write constants
	fprintf(fd, "%d,%d\n", T, eps/10);
	fflush(fd);
}

/*
 * Functions for unit conversion before printing.
 * Subject to change, but the currently-used units are:
 * Positions: Decimeters
 * Velocity: Decimeters/second
 * Accelerations: Centimeters/second^2
 * Times: milliseconds
 * Curvatures: centi-(meters^-1)
 * 
 * These units are so odd because we want decent precision without
 * ever overflowing a 32-bit integer. This is also why they're abstracted
 * away and easy to change
*/
int pos(double d) { return (int)(10.0*d); }
int vel(double d) { return (int)(10.0*d); }
int acc(double d) { return (int)(100.0*d); }
int tim(double d) { return (int)(1000.0*d); }
int curv(double d) {return (int)(100.0*d); }

/* Currently-UNUSED units, for posterity:
* Direction vectors: Magnitude 10.
*  But if we ever revive this, sqrt(325) and sqrt(65) have better
*  integrality properties.
* Distance tolerances: decimeters */
int dir(double d) { return (int)(10.0*d); }
int tol(double d) { return (int)(10.0*d); }

/* Print sensor values (to already-opened fd)*/
void printSensors(int t, int v, int xg, int yg) {
	fprintf(fd,"%d,%d,%d,%d,", t, v, xg, yg);	
}

/* Print external controller values (to already-opened fd)*/
void printCtrl(int a, int k, int t, int vh, int vl, int xg, int yg) {
	fprintf(fd,"%d,%d,%d,%d,%d,%d,%d\n", a, k, t,vh, vl, xg, yg);	
	fflush(fd);
}

/* BEGIN IMPLEMENTATION OF ENVIRONMENTS/LEVELS */
// Desired radius of turns
const int RADIUS_CM = 250;

// Construct the "rectangular" level
void CarPawnSimApi::loadRect() {
	double rad = ((double)RADIUS_CM) / 100.0;
	plan_.lineTo(rad, 0.0, 0.0, 0.00, 27.0, 0.5, 3.0);
	plan_.arcTo(3.0, 3.0, 30.0, 3.0, 27.0, 0.0, 27.0, 3.0, 4.5);
	plan_.lineTo(rad, 3.0, 30.0, 149.0, 30.0, 3.5, 4.5);
	plan_.arcTo(3.0, 152.0, 27.0, 149.0, 27.0, 149.0, 30.0, 4.0, 6.0);
	plan_.lineTo(rad, 152.0, 27.0, 152.0, -149.0, 4.0, 6.0);
	plan_.arcTo(3.0, 149.0, -152.0, 149.0, -149.0, 152.0, -149.0, 4.0, 6.0);
	plan_.lineTo(rad, 149.0, -152.0, 3.0, -152.0, 4.0, 6.0);
	plan_.arcTo(3.0, 0.0, -149.0, 3.0, -149.0, 3.0, -152.0, 4.0, 6.0);
	plan_.lineTo(rad, 0.0, -149.0, 0.00, 0.0, 4.0, 6.0);
}

// Helper function to scale plan up/down. Useful when you want to tweak 
// the level's scale because the car needs space to turn
void CarPawnSimApi::gridTo(double a, double b, double c, double d, double e, double f, double g) {
	double rate = 0.3;
	plan_.lineTo(a, b * rate, c*rate, d*rate, e*rate, f, g);
}

// Helper function to scale plan up/down. Useful when you want to tweak 
// the level's scale because the car needs space to turn
void CarPawnSimApi::aTo(double r, double cornX, double cornY, double fromX, double fromY, double toX, double toY, double f, double g) {
	double rate = 0.3;
	plan_.arcTo(r, cornX*rate + toX,   cornY*rate + toY, 
		           cornX*rate,         cornY*rate, 
		           cornX*rate + fromX, cornY*rate + fromY, f, g);
}

// Construct the "grids" level, which is all right-angle turns, but longer,
// tighter, more complex than the Rectangle level
void CarPawnSimApi::loadGrids() {
	double rad = ((double)RADIUS_CM) / 100.0;
	double margin = 3.0; // Turning area
	gridTo(rad, 0.0, 0.0, 0.0, 115.0 - margin, 2.0, 3.0);
	aTo(rad, 0.0, 115.0, 0, -margin, margin, 0, 2.0, 3.0);
	gridTo(rad, margin, 115.0,     300.0-margin, 115.0, 2.0, 3.0);
	aTo(rad, 300.0, 115.0, -margin, 0, 0, -margin, 2.0, 3.0);
	gridTo(rad, 300.0, 115.0-margin,   300.0, margin, 2.0, 3.0);
	aTo(rad, 300.0, 0, 0, margin, -margin, 0, 2.0, 3.0);
	gridTo(rad, 300.0-margin, 0.0,  200.0+margin, 0.0, 2.0, 3.0);
	aTo(rad, 200, 0, margin, 0, 0, margin, 2.0,3.0);
	gridTo(rad, 200.0, margin,     200.0, 100.0-margin,2.0,3.0);
	aTo(rad, 200, 100, 0, -margin, -margin, 0,2.0,3.0);
	gridTo(rad, 200.0-margin, 100.0,   100.0+margin, 100.0,2.0,3.0);
	aTo(rad, 100, 100, margin, 0, 0, -margin,2.0,3.0);
	gridTo(rad, 100.0, 100.0-margin,   100.0, 0.0+margin,1.0,2.0);
	aTo(rad, 100, 0, 0, margin, margin, 0,1.0,2.0);
	gridTo(rad, 100.0, margin, 200.0 - margin, 0.0,1.0,2.0);
	aTo(rad, 200, 0, -margin, 0, 0, -margin,1.0,2.0);
	gridTo(rad, 200.0, -margin, 200.0, -200.0 + margin,1.0,2.0);
	aTo(rad, 200.0, -200.0, 0.0, margin, -margin, 0.0,1.0,2.0);
	gridTo(rad, 200.0-margin, -200.0,  -100.0+margin, -200.0,1.0,2.0);
	aTo(rad, -100, -200, margin, 0, 0, margin,1.0,2.0);
	gridTo(rad, -100.0, -200.0+margin, -100.0, -100.0-margin,2.0,3.0);
	aTo(rad, -100, -100, 0, -margin, margin, 0,2.0,3.0);
	gridTo(rad, -100.0+margin, -100.0, -margin, -100.0,2.0,3.0);
	aTo(rad, 0, -100, -margin, 0, 0, margin,2.0,3.0);
	gridTo(rad, 0.0, -100.0+margin, 0.0, 0.0,2.0,3.0);
}

void CarPawnSimApi::parcTo(double rad, pt2 to, pt2 c, pt2 from, bool cW) {
	double OFFX = -200.0;
	double OFFY = -100.0;
	plan_.arcTo(rad, to.x+OFFX, to.y+OFFY, c.x+OFFX, c.y+OFFY, from.x+OFFX, from.y+OFFY, 3.0, 10.0, cW);
}
// Construct the "4-leaf Clover" level, which is all arcs
void CarPawnSimApi::loadClover() {
	double rad = 8.0;//((double)RADIUS_CM) / 100.0;
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
	
	parcTo(rad, pti, pt,  ptrl, true);
	parcTo(rad, ptlr, pt, pti, true);

	parcTo(rad, ptlt, ptl, ptlr, false);
	parcTo(rad, ptll, ptl, ptlt, false);
	parcTo(rad, ptlb, ptl, ptll, false);

	parcTo(rad, pli, pl, ptlb, true);
	parcTo(rad, pblt, pl, pli, true);
	
	parcTo(rad, pbll, pbl, pblt, false);
	parcTo(rad, pblb, pbl, pbll, false);
	parcTo(rad, pblr, pbl, pblb, false);

	parcTo(rad, pbi, pb,  pblr, true);
	parcTo(rad, pbrl, pb, pbi, true);

	parcTo(rad, pbrb, pbr, pbrl, false);
	parcTo(rad, pbrr, pbr, pbrb, false);
	parcTo(rad, pbrt, pbr, pbrr, false);

	parcTo(rad, pri, pr, pbrt, true);
	parcTo(rad, ptrb, pr, pri, true);
}

/* BEGIN CONTROL */
auto ACCEL_MAX = 0.35f; // m/s^2
auto BRAKE_MAX = 0.15f; // m/s^2
int A = 35, B = 15; // cm/s^2, derived from test data
int CIR_TOL = RADIUS_CM; // cm

// UNUSED CONSTANTS
int DIR_TOL = 10;
int X_TOL = 10;
int Y_TOL = 10;


/* Constructor for simulation object. Most "global" variables should be 
 * members initialized here, so that level stopping/restarting behaves 
 * reasonably */
CarPawnSimApi::CarPawnSimApi(const Params& params,
	const CarPawnApi::CarControls&  keyboard_controls, UWheeledVehicleMovementComponent* movement)
	: PawnSimApi(params),
	keyboard_controls_(keyboard_controls)
{
	// To track how long each system cycle actually takes
	lastTime_ = FPlatformTime::Seconds();
	// Controller mode, e.g. testing vs. follow a plan
	mode_ = PLAN;
	// Which low-level feedback controller?
	fb_ = PD;
	// Which level/environment?
	level_ = LCLOVER;
	// What node are we currently following in the plan? None!
	curNode_ = -1;
	curND_ = {};
	
	// Boilerplate
	createVehicleApi(static_cast<ACarPawn*>(params.pawn), params.home_geopoint);
	joystick_controls_ = CarPawnApi::CarControls();

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
	// Log the initial constants
	int FPS_est = GAverageFPS;
	int MSPF_est = 1000 / FPS_est;
	printConsts(MSPF_est, CIR_TOL/10);
	// Compute and log initial control values
	auto accel = 0; // Not accelerating yet
	pt2 goal = { 0.0,30.0 }; // Initial goal point, TODO: change for each level
	// Velocity limits *by time we reach waypoint*
	double vLo = curND_.vlo, vHi = curND_.vhi;
	// *K*urvature
	double k = curND_.isArc ? 1.0 / curND_.signedRad() : 0.0;
	int t = 0; // Time is relative to start of simulation!
	if (COORDINATES_RELATIVE) {
		pt2 gRel = pos2 - goal;
		printCtrl(acc(accel), curv(k), t, vel(vHi),vel(vLo), pos(0), pos(-gRel.y));
	} else {
		printCtrl(acc(accel),curv(k),t,vel(vHi),vel(vLo), pos(goal.x), pos(goal.y));
	}
	// Currently, all the levels are circular/loopable!
	plan_.connect(plan_.last(), 0);
}

void CarPawnSimApi::createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint)
{
    vehicle_api_ = std::unique_ptr<CarApiBase>(new CarPawnApi(pawn, getPawnKinematics(), home_geopoint));
}

// This is for Unreal Engine's OWN logging mechanism, so you can mostly
// ignore it - we rolled our own.
std::string CarPawnSimApi::getRecordFileLine(bool is_header_line) const
{
    std::string common_line = PawnSimApi::getRecordFileLine(is_header_line);
    if (is_header_line) {
        return common_line +
               "Throttle\tSteering\tBrake\tGear\tHandbrake\tRPM\tSpeed\t";
    }

    const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();
    const auto state = vehicle_api_->getCarState();

    common_line
        .append(std::to_string(current_controls_.throttle)).append("\t")
        .append(std::to_string(current_controls_.steering)).append("\t")
        .append(std::to_string(current_controls_.brake)).append("\t")
        .append(std::to_string(state.gear)).append("\t")
        .append(std::to_string(state.handbrake)).append("\t")
        .append(std::to_string(state.rpm)).append("\t")
        .append(std::to_string(state.speed)).append("\t")
        ;

    return common_line;
}

// Event handler for rendering
void CarPawnSimApi::updateRenderedState(float dt)
{
    PawnSimApi::updateRenderedState(dt);
    
    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}

// Another handler
void CarPawnSimApi::updateRendering(float dt)
{
    PawnSimApi::updateRendering(dt);

    updateCarControls();

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

// Main control function! Handles I/O n'At too
void CarPawnSimApi::updateCarControls()
{
	auto rc_data = getRCData();
	// Get position and velocity of car
	auto st = vehicle_api_->getCarState();
	auto pvec = //-st.kinematics_estimated.twist.linear;
		st.kinematics_estimated.pose.position;
	auto orientation = st.kinematics_estimated.pose.orientation;
	auto vvec = orientation._transformVector({ (float)st.speed,0.0f,0.0f });
	auto dvec = vvec;
	pt2 dir2 = pt2(dvec.x(), dvec.y()).unit();
	pt2 pos2(pvec.x(), pvec.y());
	auto speed = st.speed;
	auto vx = vvec[0], vy = vvec[1], vz = vvec[2];
	
	// time delta
	double newTime = FPlatformTime::Seconds();
	double ep = (newTime - lastTime_);
	lastTime_ = newTime;
	
	switch (mode_) {
	// Manual keyboard-based driving. Irrelevant for autonomous driving,
    // Useful for developing new environments / playing around
	case KEYBOARD: {
		// Print angular velocity ON-SCREEN for useful debugging
		char buf[256];
		snprintf(buf, 256, "%f", st.kinematics_estimated.twist.angular.z());
		UAirBlueprintLib::LogMessageString("Omega: ", buf, LogDebugLevel::Informational);

		// More ON-SCREEN debug info
		if (rc_data.is_initialized) {
			if (!rc_data.is_valid) {
				UAirBlueprintLib::LogMessageString("Control Mode: ", "[INVALID] Wheel/Joystick", LogDebugLevel::Informational);
				return;
			}
			UAirBlueprintLib::LogMessageString("Control Mode: ", "Wheel/Joystick", LogDebugLevel::Informational);

			// Nonsense for fancy hardware - ignore
			if (rc_data.vendor_id == "VID_044F") {
				joystick_controls_.steering = rc_data.yaw;
				joystick_controls_.throttle = (-rc_data.right_z + 1) / 2;
				joystick_controls_.brake = rc_data.throttle;

				auto car_state = vehicle_api_->getCarState();
				float rumble_strength = 0.66 + (car_state.rpm
					/ car_state.maxrpm) / 3;
				float auto_center = (1.0 - 1.0 / (std::abs(car_state.speed / 120) + 1.0))
					* (rc_data.yaw / 3);
				setRCForceFeedback(rumble_strength, auto_center);
			} 
			// Anything else, typically Logitech G920 wheel
			else {
				joystick_controls_.steering = (rc_data.throttle * 2 - 1) * 1.25;
				joystick_controls_.throttle = (-rc_data.roll + 1) / 2;
				joystick_controls_.brake = -rc_data.right_z + 1;
			}
			//Two steel levers behind wheel
			joystick_controls_.handbrake = (rc_data.getSwitch(5)) | (rc_data.getSwitch(6)) ? 1 : 0;

			// Nonsense for fancy hardware - ignore
			if ((rc_data.getSwitch(8)) | (rc_data.getSwitch(1))) { //RSB button or B button
				joystick_controls_.manual_gear = -1;
				joystick_controls_.is_manual_gear = true;
				joystick_controls_.gear_immediate = true;
			}
			else if ((rc_data.getSwitch(9)) | (rc_data.getSwitch(0))) { //LSB button or A button
				joystick_controls_.manual_gear = 0;
				joystick_controls_.is_manual_gear = false;
				joystick_controls_.gear_immediate = true;
			}

			current_controls_ = joystick_controls_;
		}
		else {
			// Main case of keyboard control
			UAirBlueprintLib::LogMessageString("Control Mode: ", "Keyboard", LogDebugLevel::Informational);
			current_controls_ = keyboard_controls_;
		}

		//if API-client control is not active then we route keyboard/joystick control to car
		if (!vehicle_api_->isApiControlEnabled()) {
			//all car controls from anywhere must be routed through API component
			vehicle_api_->setCarControls(current_controls_);
		}
		else {
			UAirBlueprintLib::LogMessageString("Control Mode: ", "API", LogDebugLevel::Informational);
			current_controls_ = vehicle_api_->getCarControls();
		}
		break;
	}
    // Autonomous controller that follows the plan!!
	case PLAN: {
		// Notify planner of new position
		plan_.setMob(pvec[0], pvec[1], vvec[0], vvec[1]);
		// Ask planner for current or next waypoint
		if (-1 == curNode_) {
			curNode_ = plan_.getCurNode();
		} else if (curND_.atEnd(pos2)) {
			auto succs = plan_.getSuccs(curNode_);
			curNode_ = succs[0];
		}
		plan_.getNode(curNode_, curND_);

		pt2 way = curND_.end;
		pt2 wayDiff = curND_.tangentAt(pos2, curND_.isCcw);
		//pt2 wayDiff = (way - pos2).unit(); // relative
		// Distance until waypoint ENTERED
		// TODO: Consider using NodeDatum.distance() function here
		auto dist = (pt2(pvec[0],pvec[1]) - way).mag() - (0.01 * RADIUS_CM);
		// Velocity after one timestep acceleration, RELATIVE to target vel
		auto vv = (st.speed + ACCEL_MAX * ep) - curND_.targetVelocity();
		// Distance after one timestep
		auto dd = dist - (ep * st.speed + ep * ep * 0.5f * ACCEL_MAX);
		// Do we need to start braking?
		// TODO: Braking is currently bang-bang, would be great to add PD
		double HARD_LIMIT = 8.0;
		bool close = ((vv * vv >= dd / (2.0f * BRAKE_MAX))) || st.speed > HARD_LIMIT;
		// For bang-bang steering
		bool goLeft = wayDiff.isLeftOf(dir2);

		// Data structure for storing/reporting control decision
		CarPawnApi::CarControls ai_controls = {};
		ai_controls.brake = 0.0f;
		ai_controls.gear_immediate = true;
		ai_controls.manual_gear = 0;
		ai_controls.is_manual_gear = false;
		
		// Which control mode? Main control computation here
		switch (fb_) {
		case PD: {
			char buf[256];
			// PD controller uses distance-to-path as proportional
			// and orientation as "derivative" to control steering
			double d = curND_.distance(pos2);
			double leftD = wayDiff.sin2(dir2);// .sin2(doubleUnit);
			auto cos = wayDiff.cos2(dir2);
			auto sq = sqrt(1 - cos * cos);
			bool l = wayDiff.isLeftOf(dir2);
			snprintf(buf, 256, "Cos: %2.2f, Sq: %2.2f, L: %d", cos, sq, l);
			UAirBlueprintLib::LogMessageString("STEER: ", buf, LogDebugLevel::Informational);

			double leftP = (l) ? -d : d;
			//goLeft = (leftD >= 0);
			double P = 0.8;
			double D = 2.015;
			double steer = P * leftP + D * leftD;
			ai_controls.steering = steer;
			// braking still bangbang
			ai_controls.throttle = close ? 0.0f : 0.99f;
			ai_controls.brake = close ? 1.0f : 0.0f;
			//On-screen debugging
			snprintf(buf, 256, "%2.2f*%2.2f=%2.2f,\t%2.2f*%2.2f=%2.2f,\tsum=%2.2f", P,leftP,P*leftP,D,leftD,D*leftD,steer);
			UAirBlueprintLib::LogMessageString("PD: ", buf, LogDebugLevel::Informational);
			break;
		}
		case BANGBANG: { // bang-bang control of both steering and braking
			const double MAX_STEER = 1.0;
			ai_controls.steering = goLeft ? -MAX_STEER : MAX_STEER;
			ai_controls.throttle = close ? 0.0f : 0.99f;
			break;
		    }
		}
		current_controls_ = ai_controls;

		// Logging code
		double vxy = vvec[0]*way.y;
		double xvy = way.x * vvec[1];
		double a = (ai_controls.throttle * (0.01*(ACCEL_MAX + BRAKE_MAX))) - (0.01*BRAKE_MAX);
		double k = curND_.isArc ? 1.0 / curND_.signedRad() : 0.0;
		double vLo = curND_.vlo, vHi = curND_.vhi;
		if (COORDINATES_RELATIVE) {
			pt2 g = (way-pos2).rebase(dir2); // Translates to vehicle-oriented coordinates
			/* *K*urvature */
			printSensors(tim(ep), vel(speed), pos(g.x), pos(g.y));
			printCtrl(acc(a), curv(k),0, vel(vHi), vel(vLo), pos(g.x), pos(g.y));
		} else {
			pt2 g = way;
			printSensors(tim(ep), vel(speed), pos(g.x), pos(g.y));
			printCtrl(acc(a), curv(k), 0, vel(vHi), vel(vLo), pos(g.x), pos(g.y));
		}
		
		// On-screen log
		char buf[256];
		snprintf(buf, 256, "%d: (%f, %f)", vvec[0] >= 0, vxy, xvy);
		UAirBlueprintLib::LogMessageString("Str: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "(%f, %f)", pvec[0], pvec[1]);
		UAirBlueprintLib::LogMessageString("Pos: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "(%f, %f)", vvec[0], vvec[1]);
		UAirBlueprintLib::LogMessageString("Vel: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "from %d(%f,%f) to (%f,%f), left:%d", curNode_, curND_.start.x, curND_.start.y, way.x, way.y, goLeft);
		UAirBlueprintLib::LogMessageString("Nav: ", buf, LogDebugLevel::Informational);
		
		if (!vehicle_api_->isApiControlEnabled()) {
			//all car controls from anywhere must be routed through API component
			vehicle_api_->setCarControls(current_controls_);
		}
		else {
			UAirBlueprintLib::LogMessageString("Control Mode: ", "API", LogDebugLevel::Informational);
			current_controls_ = vehicle_api_->getCarControls();
		}

		break;
	}
		
	case  SPIN: { // Test mode: Spin in circles, see how fast accel+brake on curve are
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
			UAirBlueprintLib::LogMessageString("Control Mode: ", "API", LogDebugLevel::Informational);
			current_controls_ = vehicle_api_->getCarControls();
		}

		break; 
	}
	default:
		break;
	}
	// More screen logging
    UAirBlueprintLib::LogMessageString("Accel: ", std::to_string(current_controls_.throttle), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Break: ", std::to_string(current_controls_.brake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Steering: ", std::to_string(current_controls_.steering), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Handbrake: ", std::to_string(current_controls_.handbrake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Target Gear: ", std::to_string(current_controls_.manual_gear), LogDebugLevel::Informational);
}

//*** Start: UpdatableState implementation ***//
void CarPawnSimApi::reset()
{
    PawnSimApi::reset();

    vehicle_api_->reset();
}

//physics tick
void CarPawnSimApi::update()
{
    vehicle_api_->update();

    PawnSimApi::update();
}

void CarPawnSimApi::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = getUUPosition();
    reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
}
//*** End: UpdatableState implementation ***//

