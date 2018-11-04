#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "CarPawnApi.h"
#include <exception>

using namespace msr::airlib;
//char buf[256] = {0};
FILE* fd = NULL;
// UAirBlueprintLib::LogMessageString(
void printConsts(int A, int B, int cirTol, int dirTol, int xTol, int yTol) {
	if (!fd) {
		fd = fopen("C:\\Users\\Brandon\\Documents\\out.csv", "w");
		if (!fd) fd = stdout;
	}
	fprintf(fd, "dx(A),dy(B),v(cirTol),xg(dirTol),yg(xTol),a(yTol),dx,dy,w,xg,yg\n");
	fflush(fd);
	//std::cout << buf << std::endl;
	//UAirBlueprintLib::LogMessageString("CSV: ", buf, LogDebugLevel::Informational);
	fprintf(fd, "%d,%d,%d,%d,%d,%d\n", A, B, cirTol, dirTol, xTol, yTol);
	//std::cout << buf << std::endl;
	fflush(fd);
	//UAirBlueprintLib::LogMessageString("CSV: ", buf, LogDebugLevel::Informational);
	//buf[0] = '\0';
 	//fflush(stdout);
}

void skipCtrls() {
	fprintf(fd, ",,,,,\n");
	fflush(fd);
}

void printSensors(int dx, int dy, int v, int xg, int yg) {
	fprintf(fd,"%d,%d,%d,%d,%d,", dx, dy, v, xg, yg);
	//std::cout << buf << std::endl;
	//UAirBlueprintLib::LogMessageString("CSV: ", buf, LogDebugLevel::Informational);
	//buf[0] = '\0';
}

void printCtrl(int a, int dx, int dy, int w, int xg, int yg) {
	fprintf(fd,"%d,%d,%d,%d,%d,%d\n", a, dx, dy, w, xg, yg);	
	fflush(fd);
}

const int RADIUS_CM = 250;

CarPawnSimApi::CarPawnSimApi(const Params& params,
	const CarPawnApi::CarControls&  keyboard_controls, UWheeledVehicleMovementComponent* movement)
	: PawnSimApi(params),
	keyboard_controls_(keyboard_controls)
{
	createVehicleApi(static_cast<ACarPawn*>(params.pawn), params.home_geopoint);
	double rad = ((double)RADIUS_CM) / 100.0;
	joystick_controls_ = CarPawnApi::CarControls();
	// car position and driveable area are from unreal editor level...
	plan_.jumpMob(0.0, 0.0); // m
	/*NodeDatum startDatum = { {0.0, 0.0}, {0.0,1.0}, {0.0,0.5}, false, RADIUS_CM / 100 };
	plan_.addNode(startDatum);*/
	/*
	plan_.lineTo(rad, 67.00, 30.50, 67.00, -150.45);
	plan_.lineTo(rad, 212.80, 31.80, 67.00, 30.50);
	plan_.lineTo(rad, 212.7,-150.2,212.8,31.80);
	plan_.lineTo(rad, 67.00,-150.45,212.7,-150.2);*/

	/*	plan_.lineTo(rad, 0.00, 0.0, 0.00, -180.95);
	plan_.lineTo(rad, 145.80, 1.30, 0.00, 0);
	plan_.lineTo(rad, 145.7,-180.7, 145.8,1.30);
	plan_.lineTo(rad, 0.00,-180.95, 145.7,-180.7);
*/
	plan_.lineTo(rad, 0.0,   0.0,    0.00,   27.0);
	plan_.arcTo(3.0,   3.0,30.0,    3.0,  27.0,    0.0, 27.0);
	plan_.lineTo(rad, 3.0,  30.0,   149.0,  30.0);
	plan_.arcTo(3.0,   152.0,27.0,  149.0,27.0,    149.0, 30.0);
	plan_.lineTo(rad, 152.0,27.0,   152.0,  -149.0);
	plan_.arcTo(3.0, 149.0, -152.0, 149.0, -149.0, 152.0, -149.0);
	plan_.lineTo(rad, 149.0,-152.0, 3.0,    -152.0);
	plan_.arcTo(3.0, 0.0, -149.0, 3.0, -149.0, 3.0, -152.0);
	plan_.lineTo(rad, 0.0, -149.0, 0.00, 0.0);
	int A = 35, B = 15; // cm/s^2, derived from test data
	int cirTol = RADIUS_CM;
	int dirTol = 100;
	int xTol = 100;
	int yTol = 100;
	printConsts(A, B, cirTol, dirTol, xTol, yTol);
	// TODO: dx,dy meaning depends on line vs arc segment
	pt2 g = {0.0,30.0};
	//plan_.getWaypoint(g);
	//pt2 mRel = plan_.getM
	auto st = vehicle_api_->getCarState();
	auto pos = st.kinematics_estimated.pose.position;
	pt2 pos2(pos.x(), pos.y());
	auto speed = st.speed;
	bool rel = false;
	if (rel) {
		pt2 gRel = pos2 - g;
		pt2 d = gRel.unit();
		printSensors((int)(100 * d.x), (int)(100 * d.y), (int)(speed * 100), 0, (int)(-100 * gRel.y));
	} else {
		pt2 gRel = g - pos2;
		pt2 d = gRel.unit();
		printSensors((int)(100 * d.x), (int)(100 * d.y), (int)(speed * 100), (int)(100*g.x), (int)(100 * g.y));
	}
	skipCtrls();
	plan_.connect(plan_.last(), 0);
	int x = 2 + 2;
}

void CarPawnSimApi::createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint)
{
    //create vehicle params
    //std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_api_ = std::unique_ptr<CarApiBase>(new CarPawnApi(pawn, getPawnKinematics(), home_geopoint));
}

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

//these are called on render ticks
void CarPawnSimApi::updateRenderedState(float dt)
{
    PawnSimApi::updateRenderedState(dt);
    
    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    //TODO: do we need this for cars?
    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}
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

enum ControlMode {
	KEYBOARD = 0,
	PLAN = 1,
	SPIN = 2,
	NUM_MODES
};

enum FeedbackMode {
	PD = 0,
	BANGBANG = 1
};

static ControlMode g_mode = PLAN;
static FeedbackMode g_fb = PD;
//const bool RUN_AI = true;
void CarPawnSimApi::updateCarControls()
{
	auto rc_data = getRCData();
	auto st = vehicle_api_->getCarState();
	auto pos = st.kinematics_estimated.pose.position;
	auto ori = st.kinematics_estimated.pose.orientation;
	auto velvec = ori._transformVector({ (float)st.speed,0.0f,0.0f });
	
	switch (g_mode) {
	case KEYBOARD: {
		//auto x = ori.matrix();
//		auto y = ori.toRotationMatrix();
	//	auto z = ori.vec();
//		auto speed = st.speed;
		//auto vx = ori[0] * speed, vy = ori[1] * speed, vz = ori[2] * speed;
		//plan_.setMob(pos[0], pos[1], vel[0], vel[1]);
		
		char buf[256];
		snprintf(buf, 256, "%f", st.kinematics_estimated.twist.angular.z());
		UAirBlueprintLib::LogMessageString("Omega: ", buf, LogDebugLevel::Informational);

		if (rc_data.is_initialized) {
			if (!rc_data.is_valid) {
				UAirBlueprintLib::LogMessageString("Control Mode: ", "[INVALID] Wheel/Joystick", LogDebugLevel::Informational);
				return;
			}
			UAirBlueprintLib::LogMessageString("Control Mode: ", "Wheel/Joystick", LogDebugLevel::Informational);

			//TODO: move this to SimModeBase?
			//if ((joystick_state_.buttons & 4) | (joystick_state_.buttons & 1024)) { //X button or Start button
			//    reset();
			//    return;
			//}

			// Thrustmaster devices
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
	case PLAN: {
		//auto vel = st.kinematics_estimated.twist.linear;
//		ori._transformVector({ speed,0.0,0.0 });
		//auto oMat = ori.matrix();
//		auto y = ori.toRotationMatrix();
	//	auto z = ori.vec();
	//plan_.setMob(pos[0], pos[1], vel[0], vel[1]);
		auto st = vehicle_api_->getCarState();
		auto pos = st.kinematics_estimated.pose.position;
		pt2 pos2(pos.x(), pos.y());
		auto speed = st.speed;
		auto vx = velvec[0], vy = velvec[1], vz = velvec[2];
		auto oreo = st.kinematics_estimated.pose.orientation;
		auto doubleStuff = oreo._transformVector({ 1.0, 0.0, 0.0 });
			
		plan_.setMob(pos[0], pos[1], velvec[0], velvec[1]);
		static int curNode = -1;
		static NodeDatum curND = {};
		if (-1 == curNode) {
			curNode = plan_.getCurNode();
		} else if (curND.atEnd(pos2)) {
			auto succs = plan_.getSuccs(curNode);
			curNode = succs[0];
		}
		plan_.getNode(curNode, curND);
		//int curNode = plan_.getCurNode();
		
		pt2 way = curND.end;
		
		/*if (curND.atEnd(pos2)) {
			if (Plan::SUCCESS != plan_.getWaypoint(way))
				break;
		}
		else {
			way = curND.end;
		}*/

		//if (Plan::SUCCESS != plan_.getWaypoint(way))
//			break;
		pt2 wayDiff = (way - pos2).unit();
		pt2 doubleUnit = pt2(doubleStuff.x(), doubleStuff.y()).unit();
		bool goLeft = wayDiff.isLeftOf(doubleUnit);
		auto acc = 0.15f;
		auto br = 0.2f;
		auto dist = (pt2(pos[0],pos[1]) - way).mag();
		auto ep = 1.0 / GAverageFPS;
		auto vv = st.speed + acc * ep;
		auto dd = dist - (ep * st.speed + ep * ep * 0.5f * acc);
		double SPEED_LIM = 16.0;
		bool close = (vv * vv >= dd / (2.0f * br)) || vv >= SPEED_LIM;
		double w = st.kinematics_estimated.twist.angular.z();
		static double defaultW = 0.0f;
		// Clean up bad data, use last value if ludicrously large
		if (w*w >= 18000.0) {
			w = defaultW;
		} else {
			defaultW = w;
		}
		//plan_.
		CarPawnApi::CarControls ai_controls = {};
		ai_controls.brake = 0.0f;
		ai_controls.gear_immediate = true;
		ai_controls.manual_gear = 0;
		ai_controls.is_manual_gear = false;
		const double MAX_STEER = 1.0;
		switch (g_fb) {
		case PD: {
			double d = curND.distance(pos2);
			double leftD = wayDiff.sin2(doubleUnit);// .sin2(doubleUnit);
			double leftP = (leftD < 0) ? -d : d;
			//double leftD = w;
			double P = 0.1;
			double D = 0.5;
			double str = P * leftP + D * leftD;
			ai_controls.steering = str;
			ai_controls.throttle = close ? 0.0f : 0.99f;
			char buf[256];
			snprintf(buf, 256, "%f\t%f", doubleUnit.sin2(wayDiff), wayDiff.sin2(doubleUnit));
			UAirBlueprintLib::LogMessageString("TESTIT: ", buf, LogDebugLevel::Informational);
			snprintf(buf, 256, "%2.2f*%2.2f=%2.2f,\t%2.2f*%2.2f=%2.2f,\tsum=%2.2f", P,leftP,P*leftP,D,leftD,D*leftD,str);
			UAirBlueprintLib::LogMessageString("PD: ", buf, LogDebugLevel::Informational);

			break;
		}
		case BANGBANG:
			ai_controls.steering = goLeft ? -MAX_STEER : MAX_STEER;
			ai_controls.throttle = close ? 0.0f : 0.99f;

			break;
		}
		current_controls_ = ai_controls;
//		char buf[256];
		//pt2 way;
		bool doit = (velvec[0] >= 0);
		double vxy = velvec[0]*way.y;
		double xvy = way.x * velvec[1];
		int A = 35, B = 15; // cm/s^2, derived from test data
		double a = (ai_controls.throttle * 50.0) - 15.0;
		bool relative = false;
	    // Angular velocity, rad/s, positive is left
		if (relative) {
			pt2 d = (curND.tangent()) * 100;
			pt2 g = way - pos2;

			printSensors((int)(100 * d.x), (int)(100 * d.y), (int)(100 * speed), (int)(100 * g.x), (int)(100 * g.y));
			printCtrl((int)a, (int)(100*d.x), (int)(100*d.y), (int)(100 * w),     (int)(100 * g.x), (int)(100 * g.y));
		} else {
			pt2 d = pt2(doubleStuff.x(), doubleStuff.y());
			pt2 g = way;
			printSensors((int)(100 * d.x), (int)(100 * d.y), (int)(100 * speed), (int)(100 * g.x), (int)(100 * g.y));
			printCtrl((int)a, (int)(100*d.x), (int)(100*d.y), (int)(100 * w),     (int)(100 * g.x), (int)(100 * g.y));
		}
		
		char buf[256];
		snprintf(buf, 256, "%d: (%f, %f)", doit, vxy, xvy);
		UAirBlueprintLib::LogMessageString("Str: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "(%f, %f)", pos[0], pos[1]/*, pos[2]*/);
		UAirBlueprintLib::LogMessageString("Pos: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "(%f, %f)", velvec[0], velvec[1]/*, velvec[2]*/);
		UAirBlueprintLib::LogMessageString("Vel: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "from %d(%f,%f) to (%f,%f), left:%d", curNode, curND.start.x, curND.start.y, way.x, way.y, goLeft);
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
		
	case  SPIN: {
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

