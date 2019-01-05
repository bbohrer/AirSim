#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "safety/Monitor.hpp"
#include <exception>
#include <set>
#include <string>

typedef double num;

// time factor
const num tf = 10.0;
// curvature factor
const num kf = 100.0;
// distance factor 
const num df = 10.0;
// eps factor
const num ef = 1.0;
// convert velocity to acceleration
const num vaf = 1.0;

// unused
//int cirTol;
//int dirTol;
//int xTol;
//int yTol;
//int dx;
//int dy;
//int w;
//int dxpost;
//int dypost;


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
int curv(double d) { return (int)(1000.0*d); }

/* Currently-UNUSED units, for posterity:
* Direction vectors: Magnitude 10.
*  But if we ever revive this, sqrt(325) and sqrt(65) have better
*  integrality properties.
* Distance tolerances: decimeters */
int dir(double d) { return (int)(10.0*d); }
int tol(double d) { return (int)(10.0*d); }

num sq(num x) { return x * x; }
num circle(num x, num y, num k) { return (k*(sq(x) + sq(y)) - 2 * x); }
// should be in deciseconds
num Monitor::brakeCycleTime(num v, num a) {
	return -(_T < ((tf*v) / -a)) ? (_T) : ((tf*v) / -a);
}
bool onUpperHalfPlane(num xg, num yg) {
	return yg >= 0;
}
bool onAnnulus(num x, num y, num k, num eps) {
	return (k*sq(eps) - 2 * eps) <= circle(x, y, k)
		&& circle(x, y, k) <= (k*sq(eps) + 2 * eps);
}

bool Monitor::controllableSpeedGoal(num v, num vl, num vh) {
	return 0 <= vl && vl <= vh && (vl <= v && v <= vh || _A * _T <= tf * df*(vh - vl) && _B*_T <= tf * df*(vh - vl));
}

//num abs(num x) { return x > 0 ? x : -x; }

void Monitor::consts(double aT, double aeps) {
	_consts = { aT,aeps };
	assert(_outfile);
	// Write header
	fprintf(_outfile, "t(T),v(eps),xg,yg,a,k,t,vh,vl,xg,yg\n");
	// Write constants
	fprintf(_outfile, "%d,%d\n", (int)(_T*10.0), (int)_eps);
	fflush(_outfile);
	aT = 0.1;
	_A = 3.40;
	_B = 1.50;
	_T = aT;
	_eps = aeps;
}

void Monitor::sense(double t, double v, double xg, double yg) {
	assert(_outfile);
	Sense rec = { t,v,{xg,yg} };
	_sense.push_back(rec);
	fprintf(_outfile, "%d,%d,%d,%d,", tim(t), vel(v), pos(xg), pos(yg));
	_tpost = t; _vpost = v; _xgpost = xg; _ygpost = yg;
	if (_vpost < 0.0) {
		_vpost = 0.0;
	}
	phys_ticks++;
	if (!plantOk()) phys_fails++;
}

void Monitor::afterSense() {
	_t = _tpost; _v = _vpost; _xg = _xgpost; _yg = _ygpost;
}

void Monitor::ctrl(double a, double k, double t, double vl, double vh, double xg, double yg) {
	assert(_outfile);
	Ctrl rec = { a,k,t,vl,vh,{xg,yg} };
	_ctrl.push_back(rec);	
	fprintf(_outfile, "%d,%d,%d,%d,%d,%d,%d\n", acc(a), curv(k), tim(t), vel(vh), vel(vl), pos(xg), pos(yg));
	_apost = a; _kpost = k; _tpost = t; _vlpost = vl; _vhpost = vh; _xgpost = xg; _ygpost = yg;
	ctrl_ticks++;
	if (!ctrlOk()) ctrl_fails++;
	bool b1 = _kpost == 0 && _xgpost == 0;
	bool b2 = _xgpost > 0 && _kpost > 0;
	bool b3 = _xgpost < 0 && _kpost < 0;
	if (!((b1)
		|| (b2)
		|| (b3)
		)) {
		case_fails++;
	}
}

void Monitor::afterCtrl() {
	_k = _kpost; _t = _tpost; _vh = _vhpost; _vl = _vlpost; _xg = _xgpost; _yg = _ygpost;
}

bool Monitor::ctrlOk(){
	num lo = _k * (sq(_xgpost) + sq(_ygpost)) - 2 * _eps;
	num mid = _kpost * (sq(_xgpost) + sq(_ygpost)) - 2 * _xgpost;
	num hi = _k * (sq(_xgpost) + sq(_ygpost)) + 2 * _eps;

	bool t1a = onUpperHalfPlane(_xgpost, _ygpost) &&
		onAnnulus(_xgpost, _ygpost, _kpost, _eps) &&
		controllableSpeedGoal(_vpost, _vlpost, _vhpost) &&
		_kpost == 0 &&
		_xgpost == 0;
	bool t2a = -_B <= _a && _a <= _A;
	bool b31a = _vpost <= _vhpost && (_a <= 0 || _a >= 0 && _vpost + _a * _T <= _vhpost);
	bool b32a = _a >= 0 &&
		(1 + 2 * _eps*_kpost + sq(_eps*_kpost))
		*((_vpost*_T
			+ _a / 2 * sq(_T))
			- ((sq(_vpost + _a * _T) - sq(_vhpost)) / (2 * _B)))
		<= (abs(_ygpost) - _eps);
	bool b33a = _a < 0 &&
		(1 + 2 * _eps*_kpost + sq(_eps*_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq(_vpost + _a * brakeCycleTime(_vpost, _a)) - sq(_vhpost)) / (2 * _B)))
		<= (abs(_ygpost) - _eps);
	bool t3a = b31a || b32a || b33a;
	bool b41a = _vlpost <= _vpost && (_a >= 0 || _a <= 0 && _vpost + _a * _T >= _vlpost);
	bool b42a = _a >= 0 && (1 + 2 * _eps*_kpost + sq(_eps*_kpost))
		*((_vpost*_T
			+ _a / 2 * sq(_T))
			+ (((sq(_vlpost) - sq(_vpost + _a * _T)))
				/ (2 * _A)))
		<= abs(_ygpost) - _eps;
	bool b43a = _a < 0 && (1 + 2 * _eps*_kpost + sq(_eps*_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a)
			+ _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq(_vlpost) - sq(_vpost + _a * brakeCycleTime(_vpost, _a)))
				/ (2 * _A)))
		<= abs(_ygpost) - _eps;
	bool t4a = b41a || b42a || b43a;
	bool c1 = t1a && t2a && t3a && t4a;

	bool t1b = onUpperHalfPlane(_xgpost, _ygpost) && onAnnulus(_xgpost, _ygpost, _kpost, _eps) && controllableSpeedGoal(_vpost, _vlpost, _vhpost) && _xgpost > 0 && _kpost > 0;
	bool t2b = -_B <= _a && _a <= _A;
	bool b31b = _vpost <= _vhpost && (_a <= 0 || _a >= 0 && _vpost + _a * _T <= _vhpost);
	bool b32b = _a >= 0 && ((1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T))
			+ ((sq(_vpost + _a * _T) - sq(_vhpost)) / (2 * _B)))) <= abs(_xgpost) - _eps;
	bool b33b = _a >= 0 && (1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T))
			+ ((sq(_vpost + _a * _T)
				- sq(_vhpost)) / (2 * _B))) <= abs(_ygpost) - _eps;
	bool b34b = _a < 0 && (1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq(_vpost + _a * brakeCycleTime(_vpost, _a))
				- sq(_vhpost)) / (2 * _B))) <= abs(_xgpost) - _eps;
	bool b35b = _a < 0 && (1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq(_vpost + _a * brakeCycleTime(_vpost, _a))
				- sq(_vhpost)) / (2 * _B)))
		<= abs(_ygpost) - _eps;
	bool t3b = b31b || b32b || b33b || b34b || b35b;
	bool b41b = _vlpost <= _vpost && (_a >= 0 || _a <= 0 && _vpost + _a * _T >= _vlpost);
	bool b42b = _a >= 0 && ((1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T))
			+ ((sq(_vlpost) - sq(_vpost + _a * _T)) / (2 * _A)))) <= abs(_xgpost) - _eps;
	bool b43b = _a >= 0 && (1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vlpost) - sq((_vpost + _a * _T))) / (2 * _A))) <= abs(_ygpost) - _eps;
	bool b44b = _a < 0 && ((1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq(_vlpost)
				- sq((_vpost*tf + _a * brakeCycleTime(_vpost, _a)))) / (2 * _A)))) <= abs(_xgpost) - _eps;
	bool b45b = _a < 0 && (1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq(_vlpost) - sq((_vpost + _a * brakeCycleTime(_vpost, _a)))) / (2 * _A))) <= abs(_ygpost) - _eps;
	bool t4b = b41b || b42b || b43b || b44b || b45b;
	bool c2 = t1b && t2b && t3b && t4b;
	bool t1c = onUpperHalfPlane(_xgpost, _ygpost) && onAnnulus(_xgpost, _ygpost, _kpost, _eps) && controllableSpeedGoal(_vpost, _vlpost, _vhpost) && _xgpost < 0 && _kpost < 0;
	bool t2c = -_B <= _a && _a <= _A;
	bool b31c = _vpost <= _vhpost && (_a <= 0 || _a >= 0 && _vpost + _a * _T <= _vhpost);
	bool b32c = _a >= 0 && (1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vpost + _a * _T) - sq(_vhpost)) / (2 * _B))) <= abs(_xgpost) - _eps;
	bool b33c = _a >= 0 && (1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T)) + ((sq((_vpost + _a * _T)) - sq(_vhpost)) / (2 * _B))) <= abs(_ygpost) - _eps;
	bool b34c = _a < 0 && ((1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq((_vpost + _a * brakeCycleTime(_vpost, _a))) - sq(_vhpost)) / (2 * _B)))) <= abs(_xgpost) - _eps;
	bool b35c = _a < 0 && (1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ ((sq((_vpost + _a * brakeCycleTime(_vpost, _a))) - sq(_vhpost)) / (2 * _B))) <= abs(_ygpost) - _eps;
	bool t3c = b31c || b32c || b33c || b34c || b35c;
	bool b41c = _vlpost <= _vpost && (_a >= 0 || _a <= 0 && _vpost + _a * _T >= _vlpost);
	bool b42c = _a >= 0 && ((1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vlpost) - sq((_vpost + _a * _T))) / (2 * _A)))) <= abs(_xgpost) - _eps;
	bool b43c = _a >= 0 && (1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vlpost) - sq((_vpost + _a * _T))) / (2 * _A))) <= abs(_ygpost) - _eps;
	bool b44c = _a < 0 && (1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
			+ (sq(_vlpost) - sq(_vpost + _a * brakeCycleTime(_vpost, _a))) / (2 * _A)) <= abs(_xgpost) - _eps;
	bool b45c = _a < 0 && (1 - 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
		*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a))) + ((sq(_vlpost) - sq((_vpost + _a * brakeCycleTime(_vpost, _a)))) / (2 * _A))) <= abs(_ygpost) - _eps;
	bool t4c = b41c || b42c || b43c || b44c || b45c;
	bool c3 = t1c && t2c && t3c && t4c;
	return c1 || c2 || c3;
}

void Monitor::ctrlErr(std::string &buf) {
	num lo =  _k     * (sq(_xgpost) + sq(_ygpost)) - 2 * _eps;
	num mid = _kpost * (sq(_xgpost) + sq(_ygpost)) - 2 * _xgpost;
	num hi = _k      * (sq(_xgpost) + sq(_ygpost)) + 2 * _eps;

	double kp = (1 + 2 * _eps*_kpost + sq(_eps*_kpost));
	double km = (1 - 2 * _eps*_kpost + sq(_eps*_kpost));
	double ay = abs(_ygpost) - _eps, ax = abs(_xgpost) - _eps;
	double delta = 0.001;
	bool c1 = onUpperHalfPlane(_xgpost, _ygpost);
	bool c2 = onAnnulus(_xgpost, _ygpost, _kpost, _eps);
	bool c3 = controllableSpeedGoal(_vpost, _vlpost, _vhpost);
	bool c4 = -_B <= _a && _a <= _A;
	bool core = c1 && c2 && c3 && c4;
	if (!core) {
		buf = "Core failed\n"; return;
	}

	if (fabs(_kpost) <= delta && fabs(_xgpost) <= delta) {
		bool simp = _vpost <= _vhpost && (_a <= delta || _a >= 0 && _vpost + _a * _T <= _vhpost);
		bool cmplx;
		if (_a >= 0) {
			double d0 = _vpost * _T;
			double dA = _a / 2 * sq(_T);
			double v1 = _vpost + _a * _T;
			double v2 = _vhpost;
			double dB = (sq(v1) - sq(v2)) / (2 * _B);
			cmplx = kp*(d0+dA+dB) <= ay;
			if (!(simp || cmplx)) {
				buf = "Failed branch 3\n"; return;
			}
		} else {
			double d0 = _vpost * brakeCycleTime(_vpost, _a);
			double dA = _a / 2 * sq(brakeCycleTime(_vpost, _a));
			double v1 = _vpost + _a * brakeCycleTime(_vpost, _a);
			double v2 = _vhpost;
			double dB = (sq(v1) - sq(v2)) / (2 * _B);
			cmplx = kp*(d0+dA+dB)<= ay;
			if (!(simp || cmplx)) {
				buf = "Failed branch 3\n"; return;
			}
		}	
		if (!(simp || cmplx)) {
			buf = "Failed branch 3\n"; return;
		}
		
		simp = _vlpost <= _vpost && (_a >= 0 || _a <= 0 && _vpost + _a * _T >= _vlpost);
		if (_a >= 0) {
			double lhs = (_vpost*_T + _a / 2 * sq(_T) + ((sq(_vlpost) - sq(_vpost + _a * _T)) / (2 * _A)));
			cmplx = lhs <= ay;
		} else {
			double lhs = (_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a))
				+ ((sq(_vlpost) - sq(_vpost + _a * brakeCycleTime(_vpost, _a))) / (2 * _A)));
			cmplx = lhs <= ay;
		}
		if (!(simp || cmplx)) {
			buf = "Failed branch 4\n"; return;
		}
	} else if (_xgpost > 0 && _kpost > 0) {
		bool simp = _vpost <= _vhpost && (_a <= 0 || _a >= 0 && _vpost + _a * _T <= _vhpost);
		bool xcmplx, ycmplx;
		if (_a >= 0) {
			xcmplx = (kp
				*((_vpost*_T + _a / 2 * sq(_T))+ ((sq(_vpost + _a * _T) - sq(_vhpost)) / (2 * _B)))) <= abs(_xgpost) - _eps;
			ycmplx = kp
				*((_vpost*_T + _a / 2 * sq(_T))
					+ ((sq(_vpost + _a * _T)- sq(_vhpost)) / (2 * _B))) <= abs(_ygpost) - _eps;
		}
		else {
			xcmplx = kp
				*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
					+ ((sq(_vpost + _a * brakeCycleTime(_vpost, _a))
						- sq(_vhpost)) / (2 * _B))) <= abs(_xgpost) - _eps;
			ycmplx = (1 + 2 * _eps*_kpost + sq(_eps)*sq(_kpost))
				*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
					+ ((sq(_vpost + _a * brakeCycleTime(_vpost, _a))
						- sq(_vhpost)) / (2 * _B)))
				<= abs(_ygpost) - _eps;
		}
		if (!(xcmplx || ycmplx || simp)) {
			buf = "B branch 3 fail\n"; return;
		}
		simp = _vlpost <= _vpost && (_a >= 0 || _a <= 0 && _vpost + _a * _T >= _vlpost);
		if (_a >= 0) {
			xcmplx = (kp
				*((_vpost*_T + _a / 2 * sq(_T))
					+ ((sq(_vlpost) - sq(_vpost + _a * _T)) / (2 * _A)))) <= abs(_xgpost) - _eps;
			ycmplx = kp
				*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vlpost) - sq((_vpost + _a * _T))) / (2 * _A))) <= abs(_ygpost) - _eps;
			if (!(xcmplx || ycmplx || simp)) {
				buf = "B branch 4 fail\n"; return;
			}
		} else {
			xcmplx = (kp
				*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
					+ ((sq(_vlpost) - sq((_vpost + _a * brakeCycleTime(_vpost, _a)))) / (2 * _A)))) <= abs(_xgpost) - _eps;
			ycmplx = kp
				*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
					+ ((sq(_vlpost) - sq((_vpost + _a * brakeCycleTime(_vpost, _a)))) / (2 * _A))) <= abs(_ygpost) - _eps;
			if (!(xcmplx || ycmplx || simp)) {
				buf = "B branch 4 fail\n"; return;
			}
		}
	}
	else if (_xgpost < 0 && _kpost < 0) {
		bool simp = _vpost <= _vhpost && (_a <= 0 || _a >= 0 && _vpost + _a * _T <= _vhpost);
		bool xcmplx, ycmplx;
		if (_a >= 0) {
			double d0 = (_vpost*_T + _a / 2 * sq(_T));
			double v1 = _vpost + _a * _T, v2 = _vhpost;
			double dB = (sq(v1) - sq(v2)) / (2 * _B);
			xcmplx = km*(d0 + dB) <= ax;
			ycmplx = km*(d0 + dB) <= ay;
			if (!(xcmplx || ycmplx || simp)) {
				buf = "C branch 3 fail\n"; return;
			}
		} else {
			double d0 = _vpost * brakeCycleTime(_vpost, _a);
			double dA = _a / 2 * sq(brakeCycleTime(_vpost, _a));
			double v1 = _vpost + _a * brakeCycleTime(_vpost, _a), v2 = _vhpost;
			double dB = (sq(v1) - sq(v2)) / (2 * _B);
			xcmplx = kp*(d0 + dA + dB) <= ax;
			ycmplx = kp*(d0 + dA + dB) <= ay;
			if (!(xcmplx || ycmplx || simp)) {
				buf = "C branch 3 fail\n"; return;
			}
		}

		simp = _vlpost <= _vpost && (_a >= 0 || _a <= 0 && _vpost + _a * _T >= _vlpost);
		if (_a >= 0) {
			xcmplx = (km
				*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vlpost) - sq((_vpost + _a * _T))) / (2 * _A)))) <= abs(_xgpost) - _eps;
			ycmplx = km
				*((_vpost*_T + _a / 2 * sq(_T)) + ((sq(_vlpost) - sq((_vpost + _a * _T))) / (2 * _A))) <= abs(_ygpost) - _eps;
		}
		else {
			xcmplx = km
				*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a)))
					+ (sq(_vlpost) - sq(_vpost + _a * brakeCycleTime(_vpost,_a))) / (2 *_A)) <= abs(_xgpost) - _eps;
			ycmplx = km
				*((_vpost*brakeCycleTime(_vpost, _a) + _a / 2 * sq(brakeCycleTime(_vpost, _a))) + ((sq(_vlpost) - sq((_vpost + _a * brakeCycleTime(_vpost, _a)))) / (2 * _A))) <= abs(_ygpost) - _eps;
		} 
		if (!(xcmplx || ycmplx || simp)) {
			buf = "C branch 4 fail\n"; return;
		}
	} else {
		buf = "Case analysis failure";
	}
}


double Monitor::pathDevOf(double k, double eps, double xgpost, double ygpost) {
	num lo = k * (sq(xgpost) + sq(ygpost)) - 2 * eps;
	num mid = k * (sq(xgpost) + sq(ygpost)) - 2 * xgpost;
	num hi = k * (sq(xgpost) + sq(ygpost)) + 2 * eps;
	if (mid < lo) {
		return mid - lo;
	}
	else if (mid > hi) {
		return mid - hi;
	}
	else {
		return 0.0;
	}
}
double Monitor::pathDeviation() {
	num lo  = _k     * (sq(_xgpost) + sq(_ygpost)) - 2 * _eps;
	num mid = _kpost * (sq(_xgpost) + sq(_ygpost)) - 2 * _xgpost;
	num hi  = _k     * (sq(_xgpost) + sq(_ygpost)) + 2 * _eps;
	if (mid < lo) {
		return mid - lo;
	} else if (mid > hi) {
		return mid - hi;
	} else {
		return 0.0;
	}
}

double Monitor::velDeviation() {
	assert(0 <= vl && vl <= vh);
	double accDev   = _A * _T  - (_vh - _vl);
	double brakeDev = _B * _T  - (_vh - _vl);
	if ((_vl <= _v && _v <= _vh) || accDev <= 0.0 || brakeDev <= 0.0) {
		return 0.0;
	} else if (accDev > 0.0) {
		return -accDev;
	} else if (brakeDev > 0.0) {
		return brakeDev;
	}
	return std::numeric_limits<double>::signaling_NaN();
}

bool Monitor::isSaved() {
	return _outfile != NULL;
}

void Monitor::saveTo(char const* path) {
	_outfile = fopen(path, "w");
	if (!_outfile) _outfile = stdout;
}

double Monitor::plantFailRate() {
	return (double)phys_fails / (double)phys_ticks;
}

double Monitor::ctrlFailRate() {
	return (double)ctrl_fails / (double)ctrl_ticks;
}

double Monitor::caseFailRate() {
	return (double)case_fails / (double)ctrl_ticks;
}

size_t Monitor::age() {
	return _ctrl.size();
}

bool Monitor::plantOk() {
	num lo  = _k * (sq(_xgpost) + sq(_ygpost)) - 2 * _eps;
	num mid = _kpost * (sq(_xgpost) + sq(_ygpost)) - 2 * _xgpost;
	num hi  = _k * (sq(_xgpost) + sq(_ygpost)) + 2 * _eps;
	bool front = 0 <= _ygpost;
	bool circ = lo <= mid && mid <= hi;
	bool monotone = _xgpost * _xg <= sq(_xg);
	bool forward = 0 <= _vpost;
	bool timely = _tpost <= _T;
	bool pm = front && circ && monotone && forward && timely;
	return pm;
}


static bool plant_okie() {
	/*num lo = k * (sq(xgpost) + sq(ygpost)) - 2 * eps   *kf*df;
	num mid = kpost * (sq(xgpost) + sq(ygpost)) - 2 * xgpost*kf*df;
	num hi = k * (sq(xgpost) + sq(ygpost)) + 2 * eps   *kf*df;
	bool front = 0 <= ygpost;
	bool circ = lo <= mid && mid <= hi;
	bool monotone = xgpost * xg <= sq(xg);
	bool forward = 0 <= vpost;
	bool timely = tpost <= T;
	bool pm = front && circ && monotone && forward && timely;
	return pm;*/
	return false;
}


static bool ctrl_okie() {
	/*num lo = (k*(sq(xgpost) + sq(ygpost))) - 2 * eps*kf*df;
	num mid = kpost * (xgpost*xgpost + ygpost * ygpost) - 2 * xgpost*df*kf;
	num hi = (k*(sq(xgpost) + sq(ygpost))) + 2 * eps*kf*df;

	bool t1a = onUpperHalfPlane(xgpost, ygpost) &&
		onAnnulus(xgpost, ygpost, kpost, eps) &&
		controllableSpeedGoal(vpost, vlpost, vhpost) &&
		kpost == 0 &&
		xgpost == 0;
	bool t2a = -B <= a && a <= A;
	bool b31a = vpost <= vhpost && (a <= 0 || a >= 0 && tf*vpost + a * T <= tf * vhpost);
	bool b32a = a >= 0 &&
		(sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps*kpost))
		*((tf*vpost*T
			+ a / 2 * sq(T))
			- ((sq(tf*vpost + a * T) - sq(tf*vhpost)) / (2 * B)))
		<= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool b33a = a < 0 &&
		(sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps*kpost))
		*((tf*vpost*brakeCycleTime(vpost, a) + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(tf*vpost + a * brakeCycleTime(vpost, a)) - sq(tf*vhpost)) / (2 * B)))
		<= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool t3a = b31a || b32a || b33a;
	bool b41a = vlpost <= vpost && (a >= 0 || a <= 0 && tf*vpost + a * T >= tf * vlpost);
	bool b42a = a >= 0 && (sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps*kpost))
		*((tf*vpost*T
			+ a / 2 * sq(T))
			+ (((sq(vlpost*tf) - sq(tf*vpost + a * T)))
				/ (2 * A)))
		<= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool b43a = a < 0 && (sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps*kpost))
		*((tf*vpost*brakeCycleTime(vpost, a)
			+ a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(tf*vlpost) - sq(tf*vpost + a * brakeCycleTime(vpost, a)))
				/ (2 * A)))
		<= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool t4a = b41a || b42a || b43a;
	bool c1 = t1a && t2a && t3a && t4a;
	bool t1b = onUpperHalfPlane(xgpost, ygpost) && onAnnulus(xgpost, ygpost, kpost, eps) && controllableSpeedGoal(vpost, vlpost, vhpost) && xgpost > 0 && kpost > 0;
	bool t2b = -B <= a && a <= A;
	bool b31b = vpost <= vhpost && (a <= 0 || a >= 0 && tf*vpost + a * T <= tf * vhpost);
	bool b32b = a >= 0 && ((sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T))
			+ ((sq(vpost*tf + a * T) - sq(tf*vhpost)) / (2 * B)))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b33b = a >= 0 && (sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T))
			+ ((sq(vpost*tf + a * T)
				- sq(tf*vhpost)) / (2 * B))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool b34b = a < 0 && ((sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*tf*brakeCycleTime(vpost, a) + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(vpost*tf + a * brakeCycleTime(vpost, a))
				- sq(tf*vhpost)) / (2 * B))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf));
	bool b35b = a < 0 && (sq(ef*tf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(vpost*tf + a * brakeCycleTime(vpost, a))
				- sq(tf*vhpost)) / (2 * B)))
		<= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool t3b = b31b || b32b || b33b || b34b || b35b;
	bool b41b = vlpost <= vpost && (a >= 0 || a <= 0 && tf*vpost + a * T >= tf * vlpost);
	bool b42b = a >= 0 && ((sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T))
			+ ((sq(vlpost*tf) - sq(vpost*tf + a * T)) / (2 * A)))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b43b = a >= 0 && (sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T)) + ((sq(tf*vlpost) - sq((vpost*tf + a * T))) / (2 * A))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool b44b = a < 0 && ((sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(tf*vlpost)
				- sq((vpost*tf + a * brakeCycleTime(vpost, a)))) / (2 * A)))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b45b = a < 0 && (sq(ef*kf) + 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(tf*vlpost) - sq((vpost*tf + a * brakeCycleTime(vpost, a)))) / (2 * A))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool t4b = b41b || b42b || b43b || b44b || b45b;
	bool c2 = t1b && t2b && t3b && t4b;
	bool t1c = onUpperHalfPlane(xgpost, ygpost) && onAnnulus(xgpost, ygpost, kpost, eps) && controllableSpeedGoal(vpost, vlpost, vhpost) && xgpost < 0 && kpost < 0;
	bool t2c = -B <= a && a <= A;
	bool b31c = vpost <= vhpost && (a <= 0 || a >= 0 && tf*vpost + a * T <= tf * vhpost);
	bool b32c = a >= 0 && (sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T)) + ((sq(vpost*tf + a * T) - sq(tf*vhpost)) / (2 * B))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b33c = a >= 0 && (sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T)) + ((sq((vpost*tf + a * T)) - sq(tf*vhpost)) / (2 * B))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool b34c = a < 0 && ((sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq((vpost*tf + a * brakeCycleTime(vpost, a))) - sq(tf*vhpost)) / (2 * B)))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b35c = a < 0 && (sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq((vpost*tf + a * brakeCycleTime(vpost, a))) - sq(tf*vhpost)) / (2 * B))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool t3c = b31c || b32c || b33c || b34c || b35c;
	bool b41c = vlpost <= vpost && (a >= 0 || a <= 0 && tf*vpost + a * T >= tf * vlpost);
	bool b42c = a >= 0 && ((sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T)) + ((sq(tf*vlpost) - sq((vpost*tf + a * T))) / (2 * A)))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b43c = a >= 0 && (sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*T*tf + a / 2 * sq(T)) + ((sq(tf*vlpost) - sq((vpost*tf + a * T))) / (2 * A))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool b44c = a < 0 && ((sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(tf*vlpost) - sq((tf*vpost + a * brakeCycleTime(vpost, a)))) / (2 * A))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf));
	bool b45c = a < 0 && (sq(ef*kf) - 2 * eps*kpost*ef*kf + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a))) + ((sq(tf*vlpost) - sq((vpost*tf + a * brakeCycleTime(vpost, a)))) / (2 * A))) <= (abs(ygpost) - eps)*sq(kf)*sq(tf);
	bool t4c = b41c || b42c || b43c || b44c || b45c;
	bool c3 = t1c && t2c && t3c && t4c;
	return c1 || c2 || c3;
	*/
return false;
}
