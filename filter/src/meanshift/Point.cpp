#include <utility>  // std::pair
#include <cmath>  // std::sqrt
#include <vector>
#include <initializer_list>

#include "Point.hpp"


Point::Point(std::vector<float> values) {
    this->values = std::move(values);
}


Point::Point(std::initializer_list<float> values) {
    this->values.assign(values);
}


Point::Point(unsigned long dimensions) {
    this->values = std::vector<float>(dimensions, 0);
}


bool Point::operator==(const Point &p) const {
    return this->values == p.values;
}


bool Point::operator!=(const Point &p) const {
    return this->values != p.values;
}


Point Point::operator+(const Point &p) const {
    Point point(this->values);
    return point += p;
}


Point &Point::operator+=(const Point &p) {
    for (long i = 0; i < p.dimensions(); ++i)
        this->values[i] += p[i];
    return *this;
}


Point Point::operator-(const Point &p) const {
    Point point(this->values);
    return point -= p;
}


Point &Point::operator-=(const Point &p) {
    for (long i = 0; i < p.dimensions(); ++i)
        this->values[i] -= p[i];
    return *this;
}


Point Point::operator*(const float d) const {
    Point point(this->values);
    return point *= d;
}


Point &Point::operator*=(const float d) {
    for (long i = 0; i < dimensions(); ++i)
        this->values[i] *= d;
    return *this;
}


Point Point::operator/(const float d) const {
    Point point(this->values);
    return point /= d;
}


Point &Point::operator/=(const float d) {
    for (long i = 0; i < dimensions(); ++i)
        this->values[i] /= d;
    return *this;
}


float &Point::operator[](const long index) {
    return values[index];
}


const float &Point::operator[](const long index) const {
    return values[index];
}


unsigned long Point::dimensions() const {
    return values.size();
}


std::vector<float>::const_iterator Point::begin() const {
    return values.begin();
}


std::vector<float>::const_iterator Point::end() const {
    return values.end();
}


float Point::euclideanDistance(const Point &p) const {
    float sum = 0.0;
    for (std::pair<std::vector<float>::const_iterator, std::vector<float>::const_iterator> i(this->begin(), p.begin());
         i.first != this->end(); ++i.first, ++i.second) {
        float diff = *i.first - *i.second;
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

