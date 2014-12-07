/*!
 * @file 位置を記述するヘッダファイル
 */

#ifndef POSITION_H_
#define POSITION_H_

#define _USE_MATH_DEFINES

#include <iostream>
#include <math.h>

/*!
 * @class 位置を記述するクラス
 */
class Position{
public:

	Position(){
		mX = 0.0;
		mY = 0.0;
	}

	virtual~Position(){}

	void SetPosition(double x, double y){
		mX = x;
		mY = y;
	}


	bool operator==(const Position& rhs){
		return (mX == rhs.x()) && (mY == rhs.y());

	}

	Position operator+(const Position& rhs);
	Position operator-(const Position& rhs);
	Position operator*(const Position& rhs);
	Position operator=(const Position& rhs);


	double x() const { return mX; }
	double& x()      { return mX; }

	double y() const { return mY; }
	double& y()      { return mY; }

private:
	double mX;
	double mY;

};

#endif /* POSITION_H_ */
