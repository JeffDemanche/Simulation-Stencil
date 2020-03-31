#include "collisionobject.h"

CollisionObject::CollisionObject(){}

CollisionPlane::CollisionPlane(Vector3f point, Vector3f normal):
    m_point(point),
    m_normal(normal)
{

}

Vector3f CollisionPlane::pointIntersection(Vector3f point)
{
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_plane#Closest_point_and_distance_for_a_hyperplane_and_arbitrary_point
    float d = m_point.dot(m_normal);
    float bracketTerm = (point.dot(m_normal) - d) / (m_normal.dot(m_normal));
    Vector3f closestPoint = point - (bracketTerm * m_normal);
    Vector3f pointToSurface = closestPoint - point;

    // Angle between vector to surface and plane normal is greater than 90 degrees.
    if (pointToSurface.dot(m_normal) < 0) {
        return Vector3f::Zero();
    }
    else {
        return pointToSurface;
    }
}


CollisionSphere::CollisionSphere(Vector3f center, float radius):
    m_center(center),
    m_radius(radius)
{

}

Vector3f CollisionSphere::pointIntersection(Vector3f point)
{
    Vector3f centerToPoint = point - m_center;
    if (centerToPoint.norm() > m_radius) {
        return Vector3f::Zero();
    }
    else {
        float distToEdge = m_radius - centerToPoint.norm();
        return centerToPoint.normalized() * distToEdge;
    }
}
