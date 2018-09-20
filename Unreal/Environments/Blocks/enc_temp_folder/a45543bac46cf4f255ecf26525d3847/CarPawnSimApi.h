#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "CarPawn.h"
#include "CarPawnApi.h"
#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "safety/SphereGeoFence.hpp"
#include "safety/SafetyEval.hpp"

class CarPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;
    
public:
    virtual ~CarPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    CarPawnSimApi(const Params& params,
        const CarPawnApi::CarControls&  keyboard_controls, UWheeledVehicleMovementComponent* movement);

    virtual void reset() override;
    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;

    virtual std::string getRecordFileLine(bool is_header_line) const override;

    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    msr::airlib::CarApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

private:
    void createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint);
    void updateCarControls();

private:
    std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
    std::vector<std::string> vehicle_api_messages_;

    //storing reference from pawn
    const CarPawnApi::CarControls& keyboard_controls_;

    CarPawnApi::CarControls joystick_controls_;
    CarPawnApi::CarControls current_controls_;
	bool last_control_mode = false;
	std::shared_ptr<msr::airlib::SphereGeoFence> fence_;
	std::shared_ptr<msr::airlib::ObstacleMap> obstacle_map_;
	msr::airlib::SafetyEval safety_eval_;
	//	SafetyEval(MultirotorApiParams vehicle_params, shared_ptr<IGeoFence> fence_ptr, shared_ptr<ObstacleMap> obs_xy);
	//ObstacleMap(int ticks, bool odd_blindspots = false);
	//fence(const Vector3r& center, float radius, float max_height, float min_height, float distance_accuracy)

};
