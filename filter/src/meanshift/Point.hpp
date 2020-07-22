#ifndef MEANSHIFT_POINT_HPP
#define MEANSHIFT_POINT_HPP

#include <vector>
#include <initializer_list>

class Point {

public:
    explicit Point(std::vector<float> values);

    Point(std::initializer_list<float> values);

    explicit Point(unsigned long dimensions);

    Point() = default;

    bool operator==(const Point &p) const;

    bool operator!=(const Point &p) const;

    Point operator+(const Point &p) const;

    Point &operator+=(const Point &p);

    Point operator-(const Point &p) const;

    Point &operator-=(const Point &p);

    Point operator*(float d) const;

    Point &operator*=(float d);

    Point operator/(float d) const;

    Point &operator/=(float d);

    float &operator[](long index);

    const float &operator[](long index) const;

    unsigned long dimensions() const;

    std::vector<float>::const_iterator begin() const;

    std::vector<float>::const_iterator end() const;

    float euclideanDistance(const Point &p) const;

private:
    std::vector<float> values;
};

#endif //MEANSHIFT_POINT_HPP
