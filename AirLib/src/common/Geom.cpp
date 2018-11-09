#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
//#include "CarPawnApi.h"
#include <exception>
#include <set>
#include "common/Geom.hpp"



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
		double thS = 0.5 * M_PI - atan2(x, y);
		/*double th1 = atan2(0, 1);
		double th2 = atan2(1, 1);
		double th3 = atan2( 1,0);
		double th4 = atan2(-1, 1);
		double th5 = atan2(-1, 0);*/
		double th = thS + rads;
		while (th > M_PI * 2) th -= M_PI*2;
		while (th < 0) th += M_PI * 2;
		double x = cos(th)*mag();
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

	bool pt2::isLeftOf(pt2 v) {
		double vxy = v.x*y;
		double xvy = x * v.y;
		if (true/*v.x >= 0*/) {
			return v.x*y < x*v.y;
		}
		else {
			return v.x*y < x*v.y;
		}
	}

	pt2 pt2::proj(pt2 other) {
		return other * (((*this)*other) / (other*other));
	}

	pt2 pt2::rebase(pt2 other) {
		//double mmag = this->mag();
		double th = atan2(other.x, other.y);
		//pt2 ounit = other.unit();
		//pt2 xaxis = ounit.rot(0.5*M_PI);
		//pt2 ref = pt2(proj(ounit).mag(), proj(xaxis).mag());
		pt2 ref = rot(th);
		return { -ref.x,ref.y };
	}



