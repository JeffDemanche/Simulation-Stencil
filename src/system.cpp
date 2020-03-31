#include "system.h"

System::System()
{
    m_colliders = std::vector<shared_ptr<CollisionObject>>();
    m_tets = std::vector<Tet>();
    m_time = 0;
    m_particles = unordered_map<int, shared_ptr<Particle>>();
    m_pushForce = Vector3f::Zero();
}

void System::setTets(std::vector<Tet> particles)
{
    m_tets = particles;
}

void System::setPushForce(shared_ptr<Particle> v1, shared_ptr<Particle> v2, shared_ptr<Particle> v3, Vector3f force)
{
    m_pushVert1 = v1;
    m_pushVert2 = v2;
    m_pushVert3 = v3;
    m_pushForce = force;
}

vector<shared_ptr<Particle>> System::getPushNodes()
{
    return vector<shared_ptr<Particle>>{ m_pushVert1, m_pushVert2, m_pushVert3 };
}

Vector3f System::getPushForce()
{
    return m_pushForce;
}

unordered_map<int, shared_ptr<Particle>> System::getParticlesMap()
{
    return m_particles;
}

vector<Particle> System::getParticleListCopy()
{
    vector<Particle> parts = vector<Particle>();
    for (unsigned int i = 0; i < m_particles.size(); i++) {
        parts.push_back(*(m_particles.at(i)));
    }
    return parts;
}

void System::setParticle(int index, shared_ptr<Particle> particle)
{
    m_particles[index] = particle;
}

shared_ptr<Particle> System::getParticle(int index)
{
    return m_particles[index];
}

std::vector<Tet> System::getTets()
{
    return m_tets;
}

std::vector<shared_ptr<CollisionObject>> System::getColliders()
{
    return m_colliders;
}

void System::addCollider(shared_ptr<CollisionObject> shape)
{
    m_colliders.push_back(shape);
}
