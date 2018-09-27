#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "CarPawnApi.h"
#include <exception>

using namespace msr::airlib;

CarPawnSimApi::CarPawnSimApi(const Params& params,
	const CarPawnApi::CarControls&  keyboard_controls, UWheeledVehicleMovementComponent* movement)
	: PawnSimApi(params),
	keyboard_controls_(keyboard_controls)
{
	createVehicleApi(static_cast<ACarPawn*>(params.pawn), params.home_geopoint);

	const double rad = 12.700;
	joystick_controls_ = CarPawnApi::CarControls();
	// car position and driveable area are from unreal editor level...
	plan_.jumpMob(0.0, 0.0); // m
	NodeDatum startDatum = { {0.0, 0.0}, rad };
	plan_.addNode(startDatum);
	/*
	plan_.lineTo(rad, 67.00, 30.50, 67.00, -150.45);
	plan_.lineTo(rad, 212.80, 31.80, 67.00, 30.50);
	plan_.lineTo(rad, 212.7,-150.2,212.8,31.80);
	plan_.lineTo(rad, 67.00,-150.45,212.7,-150.2);*/
	 
	plan_.lineTo(rad, 0.00, 0.0, 0.00, -180.95);
	plan_.lineTo(rad, 145.80, 1.30, 0.00, 0);
	plan_.lineTo(rad, 145.7,-180.7, 145.8,1.30);
	plan_.lineTo(rad, 0.00,-180.95, 145.7,-180.7);
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

static ControlMode g_mode = KEYBOARD;
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
		auto speed = st.speed;
		auto vx = velvec[0], vy = velvec[1], vz = velvec[2];
	
		plan_.setMob(pos[0], pos[1], velvec[0], velvec[1]);
		pt2 way = {0,0};
		if (Plan::SUCCESS != plan_.getWaypoint(way))
			break;
		bool goLeft = way.isLeftOf({ velvec[0], velvec[1] });
		auto acc = 0.15f;
		auto br = 0.2f;
		auto dist = (pt2(pos[0],pos[1]) - way).mag();
		auto ep = 1.0 / GAverageFPS;
		auto vv = st.speed + acc * ep;
		auto dd = dist - (ep * st.speed + ep * ep * 0.5f * acc);
		bool close = vv * vv >= dd / (2.0f * br);
		//plan_.
		CarPawnApi::CarControls ai_controls = {};
		ai_controls.brake = 0.0f;
		ai_controls.steering = goLeft ? -30.0f : 30.0f;
		ai_controls.gear_immediate = true;
		ai_controls.manual_gear = 0;
		ai_controls.is_manual_gear = false;
		ai_controls.throttle = close ? 0.0f : 0.99f;
		current_controls_ = ai_controls;
		char buf[256];
		int curNode = plan_.getCurNode();
		NodeDatum curND;
		plan_.getNode(curNode, curND);
		//pt2 way;
		plan_.getWaypoint(way);
		bool doit = (velvec[0] >= 0);
		double vxy = velvec[0]*way.y;
		double xvy = way.x * velvec[1];
		
		snprintf(buf, 256, "%d: (%f, %f)", doit, vxy, xvy);
		UAirBlueprintLib::LogMessageString("Str: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "(%f, %f)", pos[0], pos[1]/*, pos[2]*/);
		UAirBlueprintLib::LogMessageString("Pos: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "(%f, %f)", velvec[0], velvec[1]/*, velvec[2]*/);
		UAirBlueprintLib::LogMessageString("Vel: ", buf, LogDebugLevel::Informational);
		snprintf(buf, 256, "from %d(%f,%f) to (%f,%f), left:%d", curNode, curND.p.x, curND.p.y, way.x, way.y, goLeft);
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

