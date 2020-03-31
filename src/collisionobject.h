#ifndef COLLIDERSHAPE_H
#define COLLIDERSHAPE_H

#include "graphics/shape.h"
#include <Eigen/StdVector>
#include <iostream>

using namespace Eigen;
using namespace std;

class CollisionObject
{
public:
    CollisionObject();

    virtual Vector3f pointIntersection(Vector3f point) = 0;
};

class CollisionPlane : public CollisionObject
{
public:
    CollisionPlane(Vector3f point, Vector3f normal);

    /**
     * Gets the vector between the point provided and the closest point on the plane,
     * or the zero vector if the point is above the plane (based on normal).
     */
    Vector3f pointIntersection(Vector3f point) override;

private:
    Vector3f m_point;
    Vector3f m_normal;
};

class CollisionSphere : public CollisionObject
{
public:
    CollisionSphere(Vector3f center, float radius);

    /**
     * Gets the vector between the point provided and the surface of the sphere, or
     * the zero vector if the point is outside the sphere.
     */
    Vector3f pointIntersection(Vector3f point) override;

private:
    Vector3f m_center;
    float m_radius;
};

#endif // COLLIDERSHAPE_H
