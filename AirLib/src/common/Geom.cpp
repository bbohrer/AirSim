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
		double thS = atan2(x, y);
		double th = thS + rads;
		while (th > M_PI * 2) th -= M_PI*2;
		while (th < 0) th += M_PI * 2;
		return { cos(th)*mag(), sin(th)*mag() };
	}
	
	double pt2::cos2(pt2  other)  {
		return ((*this)*other) / (mag()*other.mag());
	}

	double pt2::sin2(pt2  R)  {
		auto cos = cos2(R);
		return sqrt(1 - cos * cos);
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



