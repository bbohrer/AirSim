#pragma once
class pt2 {
public:
	double x; double y;


	double operator+(pt2 const R) const;

	double operator*(pt2 const other) const;

	double mag();

	double cos2(pt2 const other) const;
	double sin2(pt2 const R) const;
	bool isLeftOf(pt2 v);
};
