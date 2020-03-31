#ifndef SOLVER_H
#define SOLVER_H

#include "system.h"
#include "collisionobject.h"

class Solver
{
public:
    Solver(float incompressibility, float rigidity, float phi, float psi, float density);
    /**
     * Solves the force function given a system state and some amount of time
     * to step into the future.
     */
    void midpointStep(System system, float seconds);

    vector<vector<Vector3f>> derivEval(System system, float seconds);

private:
    float m_incompressibility;
    float m_rigidity;
    float m_phi;
    float m_psi;
    float m_density;

};

#endif // SOLVER_H
