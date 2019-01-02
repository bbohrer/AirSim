#pragma once
#include "AirBlueprintLib.h"
#include "common/Geom.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>
#include <set>

// One recording of all the sensors, in base units (seconds, m/s, meters)
struct Sense {
	double t;
	double v; 
	pt2 g;
};

// One recording of all controls, in base units
struct Ctrl {
	double a;
	double k;
	double t;
	double vl;
	double vh;
	pt2 g;
};

struct Const {
	double T; // Max time per system cycle, in seconds
	double eps; // Max deviation from path, in meters
};

class Monitor {
public:
	void consts(double T, double eps);
	void sense(double t, double v, double xg, double yg);
	void ctrl(double a, double k, double t, double vl, double vh, double xg, double yg);
	void afterSense();
	void afterCtrl();
	bool plantOk();
	bool ctrlOk();

	double pathDeviation();
	double velDeviation();
	void saveTo(char const* path);
	bool isSaved();

	double plantFailRate();
	double ctrlFailRate();
private: 
	Const _consts;
	// If it don't make dollaz then it don't make
	std::vector<Sense> _sense;
	std::vector<Ctrl>  _ctrl;
	FILE*              _outfile;

	int phys_ticks;
	int phys_fails;

	int ctrl_ticks;
	int ctrl_fails;
};
