#ifndef SYSTEM_H
#define SYSTEM_H

#include <Eigen/StdVector>
#include <unordered_map>
#include <memory>
#include "tet.h"
#include "collisionobject.h"

using namespace Eigen;
using namespace std;

class System
{
public:
    System();

    void setTets(vector<Tet> particles);

    void setPushForce(shared_ptr<Particle> v1, shared_ptr<Particle> v2, shared_ptr<Particle> v3, Vector3f force);

    vector<shared_ptr<Particle>> getPushNodes();
    Vector3f getPushForce();

    unordered_map<int, shared_ptr<Particle>> getParticlesMap();

    vector<Particle> getParticleListCopy();

    void setParticle(int index, shared_ptr<Particle> particle);
    shared_ptr<Particle> getParticle(int index);

    vector<Tet> getTets();

    vector<shared_ptr<CollisionObject>> getColliders();

    void addCollider(shared_ptr<CollisionObject> shape);

private:
    float m_time;
    vector<Tet> m_tets;
    unordered_map<int, shared_ptr<Particle>> m_particles;
    vector<shared_ptr<CollisionObject>> m_colliders;

    shared_ptr<Particle> m_pushVert1;
    shared_ptr<Particle> m_pushVert2;
    shared_ptr<Particle> m_pushVert3;

    Vector3f m_pushForce;
};

#endif // SYSTEM_H
