#include "simulation.h"

#include <iostream>
#include <set>
#include "main.h"

#include "graphics/MeshLoader.h"

using namespace Eigen;
using namespace std;

Simulation::Simulation():
    m_system(),
    m_solver(incompressibility, rigidity, phi, psi, density)
{

}

Translation3f shapeTranslation = Translation3f(0, 3, 0);

void Simulation::init()
{

    if(MeshLoader::loadTetMesh(meshFile.toStdString(), m_vertices, m_tets)) {
        for (unsigned int i = 0; i < m_vertices.size(); i++) {
            m_system.setParticle(i, make_shared<Particle>(Particle(m_vertices.at(i) + shapeTranslation.vector(), i, 1)));
        }

        std::vector<Tet> tetsList = std::vector<Tet>();
        for (Vector4i tet : m_tets) {
            shared_ptr<Particle> m1 = m_system.getParticlesMap()[tet[0]];
            shared_ptr<Particle> m2 = m_system.getParticlesMap()[tet[1]];
            shared_ptr<Particle> m3 = m_system.getParticlesMap()[tet[2]];
            shared_ptr<Particle> m4 = m_system.getParticlesMap()[tet[3]];

            tetsList.push_back(Tet(m1, m2, m3, m4, density));
        }
        m_system.setTets(tetsList);

        // Calculate which faces are on the surface.
        // Represent a face on the tet by the index of the opposite vertex.
        // A surface face is a face that has only one opposite vertex.

        m_faces = vector<Vector3i>();
        for (Tet t : tetsList) {
            shared_ptr<Particle> n1 = t.getNodes().at(0);
            shared_ptr<Particle> n2 = t.getNodes().at(1);
            shared_ptr<Particle> n3 = t.getNodes().at(2);
            shared_ptr<Particle> n4 = t.getNodes().at(3);

            Vector3i face1 = Vector3i(n1->getIndex(), n3->getIndex(), n2->getIndex());
            Vector3i face2 = Vector3i(n1->getIndex(), n2->getIndex(), n4->getIndex());
            Vector3i face3 = Vector3i(n1->getIndex(), n4->getIndex(), n3->getIndex());
            Vector3i face4 = Vector3i(n2->getIndex(), n3->getIndex(), n4->getIndex());

            vector<Vector3i> fourFaces = vector<Vector3i>{ face1, face2, face3, face4 };

            for (int faceIndex = 0; faceIndex < 4; faceIndex++) {
                Vector3i thisFace = fourFaces.at(faceIndex);
                bool faceRemoved = false;
                vector<Vector3i>::iterator faceIter = m_faces.begin();
                while (faceIter != m_faces.end()) {
                    if (facesEqual(*faceIter, thisFace)) {
                        m_faces.erase(faceIter);
                        faceRemoved = true;
                    } else {
                        ++faceIter;
                    }
                }
                if (!faceRemoved) {
                    m_faces.push_back(thisFace);
                }
            }
        }
        m_shape.init(m_vertices, m_faces, m_tets);
    }
    m_shape.setModelMatrix(Affine3f(shapeTranslation));

    initSphere();

    initGround();
}

void Simulation::update(float seconds)
{
    m_solver.midpointStep(m_system, seconds);

    for (unsigned int i = 0; i < m_vertices.size(); i++) {
        assert(m_system.getParticlesMap().count(i) == 1);
        m_vertices.at(i) = m_system.getParticlesMap()[i]->getWorldPosition() - shapeTranslation.vector();
    }
    //m_shape.init(m_vertices, m_faces, m_tets);
    m_shape.setVertices(m_vertices);
}

void Simulation::draw(Shader *shader)
{
    m_sphere.draw(shader);
    m_shape.draw(shader);
    m_ground.draw(shader);
}

// This is from this wikipedia article: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm.
bool Simulation::rayIntersectsTriangle(Vector3f rayOrigin,
                           Vector3f rayVector,
                           vector<Vector3f> inTriangle,
                           Vector3f& outIntersectionPoint,
                           float& dist)
{
    const float EPSILON = 0.0000001;
    Vector3f vertex0 = inTriangle.at(0);
    Vector3f vertex1 = inTriangle.at(1);
    Vector3f vertex2 = inTriangle.at(2);
    Vector3f edge1, edge2, h, s, q;
    float a,f,u,v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = rayVector.cross(edge2);
    a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0/a;
    s = rayOrigin - vertex0;
    u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * rayVector.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
    {
        outIntersectionPoint = rayOrigin + rayVector * t;
        dist = t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

void Simulation::zeroPush()
{
    shared_ptr<Particle> null;
    m_system.setPushForce(null, null, null, Vector3f::Zero());
}

void Simulation::castClickRay(Vector3f point, Vector3f direction, float force)
{

    Vector3f minIntersect = Vector3f::Zero();
    float minDist = 100000.f;

    shared_ptr<Particle> mp1;
    shared_ptr<Particle> mp2;
    shared_ptr<Particle> mp3;

    for (Vector3i face : m_faces) {
        Vector3f v1 = m_system.getParticlesMap()[face[0]]->getWorldPosition();
        Vector3f v2 = m_system.getParticlesMap()[face[1]]->getWorldPosition();
        Vector3f v3 = m_system.getParticlesMap()[face[2]]->getWorldPosition();

        Vector3f rayIntersect;
        float dist;
        bool intersects = rayIntersectsTriangle(point, direction, vector<Vector3f>{ v1, v2, v3 }, rayIntersect, dist);

        if (intersects && dist < minDist) {
            minIntersect = rayIntersect;
            minDist = dist;
            mp1 = m_system.getParticlesMap()[face[0]];
            mp2 = m_system.getParticlesMap()[face[1]];
            mp3 = m_system.getParticlesMap()[face[2]];
        }
    }

    if (minDist < 100000.f) {
        m_system.setPushForce(mp1, mp2, mp3, direction * force);
    } else {
        m_system.setPushForce(mp1, mp2, mp3, Vector3f::Zero());
    }
}

void Simulation::toggleWire()
{
    m_shape.toggleWireframe();
}

Vector3f spherePos = Vector3f(0, 0, 0);
float sphereRadius = 1;

void Simulation::initGround()
{
    std::vector<Vector3f> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, 0, -5);
    groundVerts.emplace_back(-5, 0, 5);
    groundVerts.emplace_back(5, 0, 5);
    groundVerts.emplace_back(5, 0, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    m_ground.init(groundVerts, groundFaces);
    m_system.addCollider(make_shared<CollisionPlane>(CollisionPlane(Vector3f(0, 0, 0), Vector3f(0, 1, 0))));
    m_system.addCollider(make_shared<CollisionSphere>(CollisionSphere(spherePos, sphereRadius)));
}

void Simulation::initSphere()
{
    vector<Vector3f> verts;
    vector<Vector4i> tets;

    if(MeshLoader::loadTetMesh(sphereFile.toStdString(), verts, tets)) {
        vector<Vector3i> faces = vector<Vector3i>();
        for (Vector4i t : tets) {

            Vector3i face1 = Vector3i(t[0], t[2], t[1]);
            Vector3i face2 = Vector3i(t[0], t[1], t[3]);
            Vector3i face3 = Vector3i(t[0], t[3], t[2]);
            Vector3i face4 = Vector3i(t[1], t[2], t[3]);

            vector<Vector3i> fourFaces = vector<Vector3i>{ face1, face2, face3, face4 };

            for (int faceIndex = 0; faceIndex < 4; faceIndex++) {
                Vector3i thisFace = fourFaces.at(faceIndex);
                bool faceRemoved = false;
                vector<Vector3i>::iterator faceIter = faces.begin();
                while (faceIter != faces.end()) {
                    if (facesEqual(*faceIter, thisFace)) {
                        faces.erase(faceIter);
                        faceRemoved = true;
                    } else {
                        ++faceIter;
                    }
                }
                if (!faceRemoved) {
                    faces.push_back(thisFace);
                }
            }
        }
        m_sphere.init(verts, faces, tets);
        Affine3f sphereTransform = Affine3f(Eigen::Translation3f(spherePos));
        m_sphere.setModelMatrix(sphereTransform);
    }

}

bool Simulation::facesEqual(Vector3i face1, Vector3i face2)
{
    for (int i = 0; i < 3; i++) {
        bool eq = false;
        for (int j = 0; j < 3; j++) {
            if (face1[i] == face2[j]) {
                eq = true;
            }
        }
        if (eq == false) {
            return false;
        }
    }
    return true;
}

Vector3f Simulation::normal(Vector3f a, Vector3f b, Vector3f c)
{
    Vector3f e1 = b - a;
    Vector3f e2 = c - b;
    return e1.cross(e2).normalized();
}
