#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "safety/Monitor.hpp"
#include <exception>
#include <set>

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

static int A;
static int B;
static int T;
static int eps;
static int a;
static int k;
static int t;
static int v;
static int vl;
static int vh;
static int xg;
static int yg;

int apost;
int kpost;
int tpost;
int vpost;
int vlpost;
int vhpost;
int xgpost;
int ygpost;


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
num circle(num x, num y, num k) { return (k*(sq(x) + sq(y)) - 2 * x*kf*df); }
// should be in deciseconds
num brakeCycleTime(num v, num a) {
	return -(T < ((tf*v) / -a)) ? (T) : ((tf*v) / -a);
}
bool onUpperHalfPlane(num xg, num yg) {
	return yg >= 0;
}
bool onAnnulus(num x, num y, num k, num eps) {
	return sq(df)*(k*sq(eps) - 2 * kf*eps) <= circle(x, y, k)
		&& circle(x, y, k) <= sq(df)*(k*sq(eps) + 2 * kf*eps);
}

bool controllableSpeedGoal(num v, num vl, num vh) {
	return 0 <= vl && vl <= vh && (vl <= v && v <= vh || A * T <= tf * df*(vh - vl) && B*T <= tf * df*(vh - vl));
}

//num abs(num x) { return x > 0 ? x : -x; }

void Monitor::consts(double aT, double aeps) {
	_consts = { aT,aeps };
	assert(_outfile);
	// Write header
	fprintf(_outfile, "t(T),v(eps),xg,yg,a,k,t,vh,vl,xg,yg\n");
	// Write constants
	fprintf(_outfile, "%d,%d\n", (int)(T*10.0), (int)eps);
	fflush(_outfile);
	A = 3.40;
	B = 1.50;
	T = aT;
	eps = aeps;
}

void Monitor::sense(double t, double v, double xg, double yg) {
	assert(_outfile);
	Sense rec = { t,v,{xg,yg} };
	_sense.push_back(rec);
	fprintf(_outfile, "%d,%d,%d,%d,", tim(t), vel(v), pos(xg), pos(yg));
	tpost = t; vpost = v; xgpost = xg; ygpost = yg;
	phys_ticks++;
	if (!plantOk()) phys_fails++;
}

void Monitor::afterSense() {
	t = tpost; v = vpost; xg = xgpost; yg = ygpost;
}

void Monitor::ctrl(double a, double k, double t, double vl, double vh, double xg, double yg) {
	assert(_outfile);
	Ctrl rec = { a,k,t,vl,vh,{xg,yg} };
	_ctrl.push_back(rec);	
	fprintf(_outfile, "%d,%d,%d,%d,%d,%d,%d\n", acc(a), curv(k), tim(t), vel(vh), vel(vl), pos(xg), pos(yg));
	apost = a; kpost = k; tpost = t; vlpost = vl; vhpost = vh; xgpost = xg; ygpost = yg;
	ctrl_ticks++;
	if (!ctrlOk()) ctrl_fails++;
}

void Monitor::afterCtrl() {
	k = kpost; t = tpost; vh = vhpost; vl = vlpost; xg = xgpost; yg = ygpost;
}
bool Monitor::plantOk() {
	num lo  = k     *(sq(xgpost) + sq(ygpost)) - 2 * eps   *kf*df;
	num mid = kpost *(sq(xgpost) + sq(ygpost)) - 2 * xgpost*kf*df;
	num hi  = k     *(sq(xgpost) + sq(ygpost)) + 2 * eps   *kf*df;
	bool front    = 0 <= ygpost;
	bool circ     = lo <= mid && mid <= hi;
	bool monotone = xgpost*xg <= sq(xg);
	bool forward  =         0 <= vpost;
	bool timely   =     tpost <= T;
	bool pm       = front && circ && monotone && forward && timely;
	return pm;
}

bool Monitor::ctrlOk() {
	num lo = (k*(sq(xgpost) + sq(ygpost))) - 2 * eps*kf*df;
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
	bool b44b = a < 0 && ((sq(ef*kf) + 2 * eps*kpost*ef + sq(eps)*sq(kpost))
		*((vpost*brakeCycleTime(vpost, a)*tf + a / 2 * sq(brakeCycleTime(vpost, a)))
			+ ((sq(tf*vlpost)
				- sq((vpost*tf + a * brakeCycleTime(vpost, a)))) / (2 * A)))) <= (abs(xgpost) - eps)*sq(kf)*sq(tf);
	bool b45b = a < 0 && (sq(ef*kf) + 2 * eps*kpost*ef + sq(eps)*sq(kpost))
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
}

double Monitor::pathDeviation() {
	return std::numeric_limits<double>::signaling_NaN();
}

double Monitor::velDeviation() {
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