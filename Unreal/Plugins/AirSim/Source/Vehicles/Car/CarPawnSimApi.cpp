/* 
 * This file is the "main" implementation for the robot logging and monitoring.
 * It also contains some test code that can be used to figure out acceleration/braking
*/

#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "safety/Monitor.hpp"
#include "CarPawnApi.h"
#include <exception>
 

/* Constructor for simulation object. Most "global" variables should be 
 * members initialized here, so that level stopping/restarting behaves 
 * reasonably */
CarPawnSimApi::CarPawnSimApi(const Params& params,
	const CarPawnApi::CarControls&  keyboard_controls, UWheeledVehicleMovementComponent* movement)
	: PawnSimApi(params),
	keyboard_controls_(keyboard_controls)
{
	/*cruise = false;
	cruiseDir = false;
	cruiseDirSet = false;

	// To track how long each system cycle actually takes
	lastTime_ = FPlatformTime::Seconds();
	// Controller mode, e.g. testing vs. follow a plan
	mode_ = PLAN;
	// Which low-level feedback controller?
	fb_ = HUMANBOX;
	// Which level/environment?
	level_ = LRECT;
	// What node are we currently following in the plan? None!
	curNode_ = -1;
	curND_ = {};
	realCurND_ = curND_;*/

	// Boilerplate
	createVehicleApi(static_cast<ACarPawn*>(params.pawn), params.home_geopoint);
	joystick_controls_ = CarPawnApi::CarControls();
}

void CarPawnSimApi::createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint)
{
    vehicle_api_ = std::unique_ptr<CarPawnApi>(new CarPawnApi(pawn, getPawnKinematics(), home_geopoint));
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
	bool isSafe = true;
	auto rc_data = getRCData();
	// Get position and velocity of car
	auto st = vehicle_api_->getCarState();
	/*auto pvec = //-st.kinematics_estimated.twist.linear;
		st.kinematics_estimated.pose.position;*/
	auto orientation = st.kinematics_estimated.pose.orientation;
	/*auto vvec = orientation._transformVector({ (float)st.speed,0.0f,0.0f });
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
	lastTime_ = newTime;*/
	
//	pt2 way = curND_.end;
	// Print angular velocity ON-SCREEN for useful debugging
	char buf[256];
	snprintf(buf, 256, "%f", st.kinematics_estimated.twist.angular.z());
		
	// More ON-SCREEN debug info
	if (rc_data.is_initialized) {
		if (!rc_data.is_valid) {
			return;
		}

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
		current_controls_ = keyboard_controls_;
	}

	//if API-client control is not active then we route keyboard/joystick control to car
	if (!vehicle_api_->isApiControlEnabled()) {
		//all car controls from anywhere must be routed through API component
		vehicle_api_->setCarControls(current_controls_);
	}
	else {
		current_controls_ = vehicle_api_->getCarControls();
	}
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

