#include "tet.h"

Particle::Particle(Vector3f pos, int index, float mass)
{
    _index = index;
    _m = pos;
    _p = pos;
    _v = Vector3f::Zero();
    _f = Vector3f::Zero();
    _mass = mass;
}

Particle::Particle(const Particle &p) {
    _index = p.getIndex();
    _m = p.getMaterialPosition();
    _p = p.getWorldPosition();
    _v = p.getVelocity();
    _f = p.getForce();
    _mass = p.getMass();
}

int Particle::getIndex() const
{
    return _index;
}

Vector3f Particle::getMaterialPosition() const
{
    return _m;
}

Vector3f Particle::getWorldPosition() const
{
    return _p;
}

Vector3f Particle::getVelocity() const
{
    return _v;
}

Vector3f Particle::getForce() const
{
    return _f;
}

float Particle::getMass() const
{
    return _mass;
}

void Particle::setPosition(Vector3f position) {
    _p = position;
}

void Particle::addPosition(Vector3f position) {
    _p += position;
}

void Particle::setVelocity(Vector3f velocity) {
    _v = velocity;
}

void Particle::addVelocity(Vector3f velocity) {
    _v += velocity;
}

void Particle::setForce(Vector3f force) {
    _f = force;
}

void Particle::addForce(Vector3f force)  {
    _f += force;
}

void Particle::setMass(float mass)
{
    _mass = mass;
}

void Particle::addMass(float mass)
{
    _mass += mass;
}

Tet::Tet(shared_ptr<Particle> node1, shared_ptr<Particle> node2, shared_ptr<Particle> node3, shared_ptr<Particle> node4, float density):
    _node1(node1),
    _node2(node2),
    _node3(node3),
    _node4(node4)
{
    Matrix3f beta = Matrix3f();
    beta.col(0) = _node1->getMaterialPosition() - _node4->getMaterialPosition();
    beta.col(1) = _node2->getMaterialPosition() - _node4->getMaterialPosition();
    beta.col(2) = _node3->getMaterialPosition() - _node4->getMaterialPosition();
    _Beta = beta.inverse();

    _volume = tetVolume();

    _node1->addMass(density * _volume / 4.f);
    _node2->addMass(density * _volume / 4.f);
    _node3->addMass(density * _volume / 4.f);
    _node4->addMass(density * _volume / 4.f);

    _normal1 = faceNormal(0);
    _normal2 = faceNormal(1);
    _normal3 = faceNormal(2);
    _normal4 = faceNormal(3);

    _area1 = faceArea(0);
    _area2 = faceArea(1);
    _area3 = faceArea(2);
    _area4 = faceArea(3);
}

void Tet::applyForce(Vector3f force)
{
    _node1->addForce(force);
    _node2->addForce(force);
    _node3->addForce(force);
    _node4->addForce(force);
}

void Tet::setForce(Vector3f force)
{
    _node1->setForce(force);
    _node2->setForce(force);
    _node3->setForce(force);
    _node4->setForce(force);
}

void Tet::zeroForces()
{
    _node1->setForce(Vector3f::Zero());
    _node2->setForce(Vector3f::Zero());
    _node3->setForce(Vector3f::Zero());
    _node4->setForce(Vector3f::Zero());
}

void Tet::applyColliders(vector<shared_ptr<CollisionObject>> colliders, float collisionCoeff)
{

    for (shared_ptr<CollisionObject> c : colliders) {
        Vector3f col1 = c->pointIntersection(_node1->getWorldPosition());
        Vector3f col2 = c->pointIntersection(_node2->getWorldPosition());
        Vector3f col3 = c->pointIntersection(_node3->getWorldPosition());
        Vector3f col4 = c->pointIntersection(_node4->getWorldPosition());

        Vector3f greatestForce = col1;
        float greatestNorm = col1.norm();
        if (col2.norm() > greatestNorm) {
            greatestForce = col2;
            greatestNorm = col2.norm();
        }
        if (col3.norm() > greatestNorm) {
            greatestForce = col3;
            greatestNorm = col3.norm();
        }
        if (col4.norm() > greatestNorm) {
            greatestForce = col4;
            greatestNorm = col4.norm();
        }

        applyForce(greatestForce * collisionCoeff);
    }
}

void Tet::applyNodeForces(float incompressibility, float rigidity, float phi, float psi)
{
    // Get face opposite node, calculate normal and area.

    Matrix3f F = deformationGradient();
    Matrix3f stress = totalStress(incompressibility, rigidity, phi, psi);

    Vector3f force1 = F * stress * _area1 * _normal1;
    Vector3f force2 = F * stress * _area2 * _normal2;
    Vector3f force3 = F * stress * _area3 * _normal3;
    Vector3f force4 = F * stress * _area4 * _normal4;

    _node1->addForce(force1);
    _node2->addForce(force2);
    _node3->addForce(force3);
    _node4->addForce(force4);
}

vector<shared_ptr<Particle>> Tet::getNodes()
{
    return vector<shared_ptr<Particle>>{_node1, _node2, _node3, _node4};
}

Vector3f Tet::x_u(Vector3f u)
{
    return P() * _Beta * (u - _node4->getMaterialPosition()) + _node4->getWorldPosition();
}

Vector3f Tet::x_dot_u(Vector3f u)
{
    return V() * _Beta * (u - _node4->getMaterialPosition()) + _node4->getVelocity();
}

Vector3f Tet::faceNormal(int oppositeNodeIndex)
{
    assert(oppositeNodeIndex >= 0 || oppositeNodeIndex <= 3);

    shared_ptr<Particle> n = _node1;
    shared_ptr<Particle> adj1 = _node2;
    shared_ptr<Particle> adj2 = _node3;
    shared_ptr<Particle> adj3 = _node4;
    if (oppositeNodeIndex == 1) {
        n = _node2;
        adj1 = _node1;
    }
    if (oppositeNodeIndex == 2) {
        n = _node3;
        adj2 = _node1;
    }
    if (oppositeNodeIndex == 3) {
        n = _node4;
        adj3 = _node1;
    }

    Vector3f e1 = adj2->getMaterialPosition() - adj1->getMaterialPosition();
    Vector3f e2 = adj3->getMaterialPosition() - adj2->getMaterialPosition();

    // Anyone's guess whether this is facing the right way, use
    // the opposite node to determine proper normal direction.
    Vector3f norm = -e1.cross(e2).normalized();
    // Any vector from adjacent vert on the face of the normal to
    // the off-face node should have angle > 90 degrees to normal.
    Vector3f toAdj = n->getMaterialPosition() - adj1->getMaterialPosition();

    if (norm.dot(toAdj) >= 0) {
        return -norm;
    } else {
        return norm;
    }
}

float Tet::faceArea(int oppositeNodeIndex)
{
    assert(oppositeNodeIndex >= 0 && oppositeNodeIndex <= 3);
    Vector3f a = _node2->getMaterialPosition();
    Vector3f b = _node3->getMaterialPosition();
    Vector3f c = _node4->getMaterialPosition();
    if (oppositeNodeIndex == 1) {
        a = _node1->getMaterialPosition();
    }
    if (oppositeNodeIndex == 2) {
        b = _node1->getMaterialPosition();
    }
    if (oppositeNodeIndex == 3) {
        c = _node1->getMaterialPosition();
    }

    //https://math.stackexchange.com/questions/507496/how-do-you-find-the-area-of-a-triangle-in-a-3d-graph
    return (b - a).cross(c - a).norm() * 0.5;
}

float Tet::tetVolume()
{
    Matrix3f mat = Matrix3f();
    mat.col(0) = _node1->getMaterialPosition() - _node4->getMaterialPosition();
    mat.col(1) = _node2->getMaterialPosition() - _node4->getMaterialPosition();
    mat.col(2) = _node3->getMaterialPosition() - _node4->getMaterialPosition();

    return abs(mat.determinant()) / 6.f;
}

Matrix3f Tet::deformationGradient()
{
    return P() * _Beta;
}

Matrix3f Tet::velocityGradient()
{
    return V() * _Beta;
}

Matrix3f Tet::P()
{
    Matrix3f P = Matrix3f();
    P.col(0) = _node1->getWorldPosition() - _node4->getWorldPosition();
    P.col(1) = _node2->getWorldPosition() - _node4->getWorldPosition();
    P.col(2) = _node3->getWorldPosition() - _node4->getWorldPosition();
    return P;
}

Matrix3f Tet::V()
{
    Matrix3f V = Matrix3f();
    V.col(0) = _node1->getVelocity() - _node4->getVelocity();
    V.col(1) = _node2->getVelocity() - _node4->getVelocity();
    V.col(2) = _node3->getVelocity() - _node4->getVelocity();
    return V;
}

Matrix3f Tet::greensStrain()
{
    Matrix3f defGradient = deformationGradient();
    return (defGradient.transpose() * defGradient) - Matrix3f::Identity();
}

Matrix3f Tet::strainRate()
{
    // 3/9 slide 9
    Matrix3f defGradient = deformationGradient();
    Matrix3f velGradient = velocityGradient();

    return (defGradient.transpose() * velGradient) + (velGradient.transpose() * defGradient);
}

Matrix3f Tet::elasticStress(float incompressibility, float rigidity)
{
    // 3/9 slide 6
    Matrix3f strain = greensStrain();
    return (incompressibility * Matrix3f::Identity() * strain.trace()) + (2 * rigidity * strain);
}

Matrix3f Tet::viscousStress(float phi, float psi)
{
    // 3/9 slide 11
    Matrix3f rate = strainRate();
    return (phi * Matrix3f::Identity() * rate.trace()) + (2 * psi * rate);
}

Matrix3f Tet::totalStress(float incompressibility, float rigidity, float phi, float psi)
{
    Matrix3f elastic = elasticStress(incompressibility, rigidity);
    Matrix3f viscous = viscousStress(phi, psi);
    return elastic + viscous;
}
