# Simulation

## Running

I'm submitting a Qt Creator project setup. It's run with the following
arguments:

<Path to .mesh file> <incompressibility> <rigidity> <phi> <psi> <density>
<Path to sphere .mesh file>

Here, incompressibility and rigidity are both for elastic stress and phi and psi
are for viscous stress, to reduce confusion. For me on Windows, the paths to the
mesh file and sphere mesh file has to be absolute.

The simulation begins paused. Press space to start simulation.

## Features/Issues

I implemented all basic features. Some notes:

 - Once the simulation begins, the wireframe view of the tets gets really
jumbled. I tried to figure out why this is happening, but I don't think it's
heavily affecting how the simulation goes.

 - Meshes collapse/explode for very low/high parameters. Generally 30-500 seems
to be a good range for the ellipsoid, where too much lower causes it to collapse
and anything higher causes it to explode. The collapsing is due to too much
force being applied at once.

 - I'm going with a gravity of 1 and a collision penalty of 10.

I also implemented pushing pulling of the mesh. You can pull the mesh by
holding down the 1 key, and can push by holding down the 2 or 3 keys (3 is twice
as strong). This will apply a force to whatever geometry is at the center of the
viewport window, in the direction that the viewer is looking.

## Video

The video I'm including is a screen capture of the ellipsoid mesh with the
following parameters:

 - incompressibility: 35
 - rigidity: 35
 - phi: 35
 - psi: 35
 - density: 35

The mesh collides with both a sphere collider and the ground plane. The actual
collider on the ground plane extends infinitely even though the rendered mesh
is finite.

Around 0:17 I start applying a pull force to make the ellipsoid come back and
interact with the sphere again.

## Code Layout

simulation.cpp - Sort of a starting place. Has member variables for system and
solver. Handles extracting surface mesh from points.

system.cpp - Holds onto data and provides access to particles and tets for the
solver.

solver.cpp - Runs midpoint integration and applies forces to tets within the
derivEval method.

tet.cpp - Object for a single tet. Handles all calculation for internal forces,
stress, strain, etc.

collisionobject.cpp - Calculates collisions on plane and sphere shapes.
