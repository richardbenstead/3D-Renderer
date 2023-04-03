#pragma once
#include "types.h"

// Calculates the distance from a point to a line
float DistanceToLine(Vec3f const &line_point1, Vec3f const &line_point2) {
    Vec3f line = line_point2 - line_point1;
    float t = line_point1.dot(line) / line.norm();
    if (t < 0) {
        return line_point1.norm();
    } else if (t > 1) {
        return line_point2.norm();
    } else {
        Vec3f projection = line_point1 + t * line;
        return projection.norm();
    }
}

Vec3f Normal(Vec3f const &p1, Vec3f const &p2, Vec3f const &p3) {
    Vec3f cross = (p2 - p1).cross(p3 - p1);
    return cross.normalized();
}

bool FacesCamera(Eigen::Matrix<float, 3, 3> const &points) {
    // Calculate the normal of the triangle
    Vec3f normal = Normal(points.col(0), points.col(1), points.col(2));
    float d = normal.dot(points.col(0));
    return d > 0;
}

float NormToPoint(Eigen::Matrix<float, 3, 3> const &points, Vec3f const &p) {
    // Calculate the normal of the triangle
    Vec3f normal = Normal(points.col(0), points.col(1), points.col(2));
    Vec3f vecToP = (p - points.col(0)).normalized();
    float facePoint = -normal.dot(vecToP);
    return facePoint;
}

float ShortestDistance(Vec3f const &t1, Vec3f const &t2, Vec3f const &t3) {
    // Calculate the normal of the triangle
    Vec3f normal = Normal(t1, t2, t3);

    // Calculate the distance from the point to the plane of the triangle
    float d = normal.dot(t1);

    // If the point is on the same side of the plane as the normal, the shortest
    // distance is the distance from the point to the plane
    if (d * normal.dot(t1) > 0) {
        return std::abs(d);
    }

    // Otherwise, the shortest distance is the distance from the point to one of the edges or vertices of the triangle
    return std::min(
        {DistanceToLine(t1, t2), DistanceToLine(t2, t3), DistanceToLine(t3, t1), t1.norm(), t2.norm(), t3.norm()});
}
