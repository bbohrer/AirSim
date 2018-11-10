#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "CarPawn.h"
#include "CarPawnApi.h"
#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "safety/Plan.hpp"
#include "physics/Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

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

enum Level {
	LRECT = 0,
	LGRIDS
};

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
	void loadRect();
	void loadGrids();
	void gridTo(double a, double b, double c, double d, double e, double f, double g);
	void aTo(double r, double cornX, double cornY, double fromX, double fromY, double toX, double toY, double f, double g);
private:
    std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
    std::vector<std::string> vehicle_api_messages_;

    //storing reference from pawn
    const CarPawnApi::CarControls& keyboard_controls_;
	Plan plan_;
    CarPawnApi::CarControls joystick_controls_;
    CarPawnApi::CarControls current_controls_;
	ControlMode mode_;
	FeedbackMode fb_;
	Level level_;
	int curNode_;
	NodeDatum curND_;
	double defaultW;
	double lastTime_;
};
