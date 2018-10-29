#pragma once
class pt2 {
public:
	double x; double y;

	pt2(double x, double y); 
	pt2();

	pt2 operator+(pt2 const R) const;
	pt2 operator-(pt2 const R) const;

	double operator*(pt2 const other) const;
	pt2 operator*(double scale) const;

	double mag();
	pt2 unit();
	pt2 rot(double rads);

	double cos2(pt2  other) ;
	double sin2(pt2  R) ;
	bool isLeftOf(pt2 v);
};

