#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
//#include "CarPawnApi.h"
#include <exception>
#include <set>
#include "common/Geom.hpp"
#include <cmath>



using namespace msr::airlib;
pt2::pt2(double xx, double yy) : x(xx), y(yy) {}
pt2::pt2() : x(0.0), y(0.0) {}
pt2 pt2::operator+(pt2 const R) const { return { x + R.x, y + R.y}; }
pt2 pt2::operator-(pt2 const R) const { return { x - R.x, y - R.y }; }

	double pt2::operator*(pt2 const other) const {
		return x*other.x + y*other.y;
	}

	pt2 pt2::operator*(double other) const {
		return { x*other, y*other };
	}

	double pt2::mag(){
		return sqrt(x*x + y*y);
	}

	pt2 pt2::unit() {
		double m = mag();
		if (m == 0.0) {
			return pt2(0.0, 0.0);
		}
		return pt2(x / m, y / m);
	}

	pt2 pt2::rot(double rads) {
		double thS = /*0.5 * M_PI - */atan2(y,x);
		double tTrans = M_PI - thS;
		double xx = cos(tTrans), yy = sin(tTrans);
		double redo = atan2(yy, xx);
		double x1 = atan2(0, 1);
		double x2 = atan2(1, 1);
		double x3 = atan2(1, 0);
		double x4 = atan2(1, -1);
		double x5 = atan2(0, -1);
		double x6 = atan2(-1, -1);
		double x7 = atan2(-1, 0);
		double x8 = atan2(-1, 1);
		/*double th1 = atan2(0, 1);
		double th2 = atan2(1, 1);
		double th3 = atan2( 1,0);
		double th4 = atan2(-1, 1);
		double th5 = atan2(-1, 0);*/
		double th = tTrans + rads;
		while (th > M_PI * 2) th -= M_PI*2;
		while (th < 0) th += M_PI * 2;
		double x = -cos(th)*mag();
		double y = sin(th)*mag();
		return { x, y };
	}
	
	double pt2::cos2(pt2  other)  {
		return ((*this)*other) / (mag()*other.mag());
	}

	double pt2::sin2(pt2  R)  {
		auto cos = cos2(R);
		auto sq = sqrt(1 - cos * cos);
		bool l = isLeftOf(R);
		return l ? -sq : sq;
	}

	inline bool q1(pt2 v) { return v.x < 0 && v.y > 0; }
	inline bool q2(pt2 v) { return v.x >= 0 && v.y > 0; }
	inline bool q3(pt2 v) { return v.x >= 0 && v.y <= 0; }
	inline bool q4(pt2 v) { return !(q1(v) || q2(v) || q3(v)); }
	inline double sl(pt2 v) { return v.y / v.x; }
	bool pt2::isLeftOf(pt2 v) {
		double vxy = v.x*y;
		double xvy = x * v.y;
		if (q1(v)) { // Quadrant I
			return(q1(*this) && sl(*this) < sl(v))
				|| q2(*this)
				|| (q3(*this) && sl(*this) > sl(v));
		} else if (q2(v)) { // Quadrant II
			return(q2(*this) && sl(*this) < sl(v))
				|| q3(*this)
				|| (q4(*this) && sl(*this) > sl(v));
		} else if (q3(v)){ // Quadrant III
			return (q3(*this) && sl(*this) < sl(v))
				|| q4(*this)
				|| (q1(*this) && sl(*this) > sl(v));
		} else { // Quadrant IV
			return (q4(*this) && sl(*this) < sl(v))
				|| q1(*this)
				|| (q2(*this) && sl(*this) > sl(v)) ;
		}
	}

	pt2 pt2::proj(pt2 other) {
		return other * (((*this)*other) / (other*other));
	}

	pt2 pt2::rebase(pt2 other) {
		//double mmag = this->mag();
		double th = atan2(other.x, other.y);
		////pt2 ounit = other.unit();
		////pt2 xaxis = ounit.rot(0.5*M_PI);
		////pt2 ref = pt2(proj(ounit).mag(), proj(xaxis).mag());
//		pt2 ref = rot(th);
		return other;//{ -ref.x,ref.y };
	}



