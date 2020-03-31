#include "solver.h"

Solver::Solver(float incompressibility, float rigidity, float phi, float psi, float density):
    m_incompressibility(incompressibility),
    m_rigidity(rigidity),
    m_phi(phi),
    m_psi(psi),
    m_density(density)
{
}

void Solver::midpointStep(System system, float seconds)
{
    // Record original node position and velocity.
    vector<vector<Vector3f>> originalPosVel = vector<vector<Vector3f>>();
    for (unsigned int i = 0; i < system.getParticlesMap().size(); i++) {
        vector<Vector3f> posVel = vector<Vector3f>();
        posVel.push_back(system.getParticlesMap()[i].get()->getWorldPosition());
        posVel.push_back(system.getParticlesMap()[i].get()->getVelocity());
        originalPosVel.push_back(posVel);
    }

    // Get position and velocity after a naive euler step.
    vector<vector<Vector3f>> eulerStep = derivEval(system, seconds);

    // Update the system object with values halway between the original and the euler destination.
    vector<Particle> parts = system.getParticleListCopy();
    for (unsigned int i = 0; i < parts.size(); i++) {
        parts.at(i).addPosition(eulerStep.at(i).at(0) * 0.5);
        parts.at(i).addVelocity(eulerStep.at(i).at(1) * 0.5 * seconds);

        system.getParticle(i).get()->addPosition(eulerStep.at(i).at(0) * 0.5);
        system.getParticle(i).get()->addVelocity(eulerStep.at(i).at(1) * 0.5 * seconds);
    }

    // Get pos and vel of the system calculated from the midpoint between original and euler.
    vector<vector<Vector3f>> midStep = derivEval(system, seconds * 0.5);

    // Calculate final position and velocity.
    for (unsigned int i = 0; i < midStep.size(); i++) {
        Vector3f finalPos = originalPosVel.at(i).at(0) + midStep.at(i).at(0);
        Vector3f finalVel = originalPosVel.at(i).at(1) + (seconds * midStep.at(i).at(1));

        system.getParticle(i).get()->setPosition(finalPos);
        system.getParticle(i).get()->setVelocity(finalVel);
    }
}


vector<vector<Vector3f>> Solver::derivEval(System system, float seconds)
{
    // Zero forces
    for (Tet tet : system.getTets()) {
        tet.zeroForces();
    }

    if (system.getPushForce() != Vector3f::Zero()) {
        for (shared_ptr<Particle> p : system.getPushNodes()) {
            p->addForce(system.getPushForce());
        }
    }

    for (unsigned int i = 0; i < system.getParticlesMap().size(); i++) {
        system.getParticlesMap()[i]->addForce(Vector3f(0, -1, 0));
    }

    // Accumulate all forces here.
    for (Tet tet : system.getTets()) {
        tet.applyColliders(system.getColliders(), 10);
        tet.applyNodeForces(m_incompressibility, m_rigidity, m_phi, m_psi);
    }

    vector<vector<Vector3f>> posVels = vector<vector<Vector3f>>();
    for (unsigned int i = 0; i < system.getParticlesMap().size(); i++) {
        shared_ptr<Particle> p = system.getParticlesMap()[i];

        vector<Vector3f> posVel = vector<Vector3f>();
        posVel.push_back(p->getVelocity());
        posVel.push_back(p->getForce() / p->getMass());

        posVels.push_back(posVel);
    }

    return posVels;
}
