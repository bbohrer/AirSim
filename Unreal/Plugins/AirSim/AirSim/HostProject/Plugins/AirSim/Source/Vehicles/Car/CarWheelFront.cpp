// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelFront.h"
#include "TireConfig.h"
#include "UObject/ConstructorHelpers.h"

UCarWheelFront::UCarWheelFront()
{
    ShapeRadius = 0.01615; //m
    ShapeWidth  = 0.025f; //m
    Mass = 2.596f; // 2.596kg
    DampingRate = 0.25f;
    bAffectedByHandbrake = false;
    SteerAngle = 30.f;

    // Setup suspension forces
    SuspensionForceOffset = 0.0f;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SuspensionNaturalFrequency = 9.0f;
    SuspensionDampingRatio = 1.05f;

    // Find the tire object and set the data for it
    static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/AirSim/VehicleAdv/Vehicle/WheelData/Vehicle_FrontTireConfig.Vehicle_FrontTireConfig"));
    TireConfig = TireData.Object;
/*vehicle.C_x = 103.94;       % longitudinal stiffness (N)
vehicle.C_alpha = 56.4;     % cornering stiffness (N)
vehicle.I_z = 0.0558;       % rotation inertia (kgm^2)
vehicle.mu = 1.37;          % friction coefficient
vehicle.mu_slide = 1.96;    % sliding friction coefficient
vehicle.L = 0.257;          % wheelbase (m)
vehicle.L_f = 0.115;        % CoG to front axle (m)
vehicle.L_r = 0.142;        % CoG to rear axle (m)
vehicle.tw = 0.165;         % trackwidth (m) */
}
