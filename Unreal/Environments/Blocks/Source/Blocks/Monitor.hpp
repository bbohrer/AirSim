#pragma once
#include "AirBlueprintLib.h"
#include "Geom.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>
#include <set>
#include <string>
#include <windows.h>

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
	Monitor();
	void consts(double T, double eps);
	void sense(double t, double v, double xg, double yg);
	void ctrl(double a, double k, double t, double vl, double vh, double xg, double yg);
	void afterSense();
	void afterCtrl();
	bool plantOk();
	bool ctrlOk();
	static double pathDevOf(double k, double eps, double xgpost, double ygpost);

	double pathDeviation();
	double velDeviation();
	void saveTo(char const* path);
	bool isSaved();

	double plantFailRate();
	double ctrlFailRate();
	double caseFailRate();
	size_t age();
	void ctrlErr(std::string &buf);
	bool _extCtrlMon;
	bool _extPlantMon;
	int b1;
	int b2;
	int b3;
	int b4;
private:
	Const _consts;
	// If it don't make dollaz then it don't make
	std::vector<Sense> _sense;
	std::vector<Ctrl>  _ctrl;
	FILE*              _outfile;

public:
	int phys_ticks;
	int phys_fails;

	int ctrl_ticks;
	int ctrl_fails;
	int case_fails;
private:
	double _A;
	double _B;
	double _T;
	double _eps;
	double _a;
	double _k;
	double _t;
	double _v;
	double _vl;
	double _vh;
	double _xg;
	double _yg;

	double _apost;
	double _kpost;
	double _tpost;
	double _vpost;
	double _vlpost;
	double _vhpost;
	double _xgpost;
	double _ygpost;
	HANDLE _pipe;


	bool controllableSpeedGoal(double v, double vl, double vh);
	double brakeCycleTime(double v, double a);

public:
	bool _velStarted;
	bool _finished;
	int _cycles;
	double _velAvg;
	void ctrlFallback(double a);
	double boundaryDist();
};