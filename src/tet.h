#ifndef TET_H
#define TET_H

#include <memory>
#include <iostream>
#include <Eigen/StdVector>
#include <cstdlib>
#include "collisionobject.h"

using namespace Eigen;
using namespace std;

class Particle
{
public:
    /**
     * @param pos The initial position of a particle. Becomes material space
     *            position and also initial world space.
     * @param index Index in the shape (corresponding with a vertex in a mesh
     *              list.
     * @param mass Mass of the particle.
     */
    Particle(Vector3f pos, int index, float mass);

    Particle(const Particle &p);

    int getIndex() const;
    Vector3f getMaterialPosition() const;
    Vector3f getWorldPosition() const;
    Vector3f getVelocity() const;
    Vector3f getForce() const;
    float getMass() const;

    void setPosition(Vector3f position);
    void addPosition(Vector3f position);
    void setVelocity(Vector3f velocity);
    void addVelocity(Vector3f velocity);
    void setForce(Vector3f force);
    void addForce(Vector3f force);
    void setMass(float mass);
    void addMass(float mass);

private:
    int _index;

    /** Position in material space. */
    Vector3f _m;

    /** Position in world space. */
    Vector3f _p;

    /** Velocity */
    Vector3f _v;

    /** Force accumulator */
    Vector3f _f;

    float _mass;
};

class Tet
{
public:
    Tet(shared_ptr<Particle> node1, shared_ptr<Particle> node2, shared_ptr<Particle> node3, shared_ptr<Particle> node4, float density);

    /**
     * Applies a force to all particles in the tet uniformly.
     */
    void applyForce(Vector3f force);

    void setForce(Vector3f force);

    /**
     * Sets all particle force accumulators to zero vector.
     */
    void zeroForces();

    /**
     * Called per step to apply appropriate forces to the tet for collision.
     */
    void applyColliders(vector<shared_ptr<CollisionObject>> colliders, float collisionCoeff);

    /**
     * Accumulates forces on each node due to stress.
     */
    void applyNodeForces(float incompressibility, float rigidity, float phi, float psi);

    vector<shared_ptr<Particle>> getNodes();

    Vector3f faceNormal(int oppositeNodeIndex);
    float faceArea(int oppositeNodeIndex);

private:
    /**
     * Transforms a vector in material space (tet in rest position) to it's
     * position in world space (tet during simulation). u in this case lies
     * somewhere on the tet (I think it could be inside even but that's not
     * necessary).
     */
    Vector3f x_u(Vector3f u);

    /**
     * Transforms a velocity vector from material space to world space. See
     * the x_u function.
     */
    Vector3f x_dot_u(Vector3f u);

    float tetVolume();

    Matrix3f deformationGradient();
    Matrix3f velocityGradient();

    Matrix3f P();
    Matrix3f V();

    Matrix3f greensStrain();
    Matrix3f strainRate();

    Matrix3f elasticStress(float incompressibility, float rigidity);
    Matrix3f viscousStress(float phi, float psi);
    Matrix3f totalStress(float incompressibility, float rigidity, float phi, float psi);

    Matrix3f _Beta;

    float _volume;

    shared_ptr<Particle> _node1;
    shared_ptr<Particle> _node2;
    shared_ptr<Particle> _node3;
    shared_ptr<Particle> _node4;

    Vector3f _normal1;
    Vector3f _normal2;
    Vector3f _normal3;
    Vector3f _normal4;

    float _area1;
    float _area2;
    float _area3;
    float _area4;

};

#endif // TET_H
