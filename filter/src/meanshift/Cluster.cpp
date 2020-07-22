#include <cmath>
#include <vector>

#include "Cluster.hpp"
#include "Point.hpp"

Cluster::Cluster(Point centroid)
{
    this->centroid = std::move(centroid);
}

Point Cluster::getCentroid() const
{
    return centroid;
}

void Cluster::addPoint(Point point, int index)
{
    points.emplace_back(point);
    origin_index.push_back(index);
}

long Cluster::getSize() const
{
    return points.size();
}

std::vector<Point>::iterator Cluster::begin()
{
    return points.begin();
}

std::vector<Point>::iterator Cluster::end()
{
    return points.end();
}

float Cluster::getSse() const
{
    float sum = 0.0;
    for (const Point &p : points)
        sum += std::pow(p.euclideanDistance(centroid), 2);
    return sum;
}
