

#include "position.h"


Position Position::operator+(const Position& rhs){

	Position tmp;
	tmp.x() = this->x() + rhs.x();
	tmp.y() = this->y() + rhs.y();

	return tmp;
}

Position Position::operator-(const Position& rhs){

	Position tmp;
	tmp.x() = this->x() + rhs.x();
	tmp.y() = this->y() + rhs.y();

	return tmp;
}


Position Position::operator*(const Position& rhs){

	Position tmp;
	tmp.x() = this->x() * rhs.x();
	tmp.y() = this->y() * rhs.y();

	return tmp;
}

Position Position::operator=(const Position& rhs){

	this->x() = rhs.x();
	this->y() = rhs.y();
	return *this;
}

