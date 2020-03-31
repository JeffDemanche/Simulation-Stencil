#ifndef SIMULATION_H
#define SIMULATION_H

#include <memory>
#include "graphics/shape.h"
#include "solver.h"
#include "system.h"

class Shader;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    bool rayIntersectsTriangle(Vector3f rayOrigin,
                               Vector3f rayVector,
                               vector<Vector3f> inTriangle,
                               Vector3f& outIntersectionPoint,
                               float& dist);

    void zeroPush();

    void castClickRay(Vector3f point, Vector3f direction, float force);

    void toggleWire();
private:
    bool facesEqual(Vector3i face1, Vector3i face2);

    Vector3f normal(Vector3f a, Vector3f b, Vector3f c);

    System m_system;
    Solver m_solver;

    vector<Vector3f> m_vertices;
    vector<Vector3i> m_faces;
    vector<Vector4i> m_tets;

    Shape m_shape;
    Shape m_sphere;

    Shape m_ground;
    void initGround();
    void initSphere();
};

#endif // SIMULATION_H
