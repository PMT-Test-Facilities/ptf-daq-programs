Prep:

• Generate annular cylinder for tank
• Generate sphere for PMT top
• Generate prisms for gantries 

Generation:

• Simple checks
  - Are final destinations outside range?
  - Will final x₀, x₁ cause collision for gantries?
  - Will final positions collide with PMT top?
  - Will final positions collide with tank wall?
• Simple path generation:
  - Tilt + rotation first
  - Can we move all pos variables at the same time?
    ⋆ Check for collision of enlarged cylinders along path
    ⋆ If collision, find parametric 𝑡 for collision then check bounding spheres for true collision
    ⋆ False positives for collisions, but no false negatives
  - If no, can we move Z after? (cylinder extrusion for XY, endpoint for Z)
    ⋆ Try each gantry first
  - If no, can we move Z before? (cylinder extrusion for XY, endpoint for Z)
    ⋆ Try each gantry first
  - If no, try each permutation of moving dimensions at the same time in XYZ, YZX, ZXY, ZYX, YXZ, XZY order
    ⋆ For each, simply need to check if other dimensions are within tolerance (if moving both in X, then Y and Z positions must be different enough)
  - If no, try each dimension individually for each gantry in every combination (endpoint for all)
  - If no, fail with no multilinear path possible with single gantry movement

Cylinder extrusion algorithm:
• Find bounding sphere at start and endpoint
• Use path between centroids to find angle on sphere, find opposing points
• Generate cylinder with radius equal to bounding spheres, going between opposing points
• Check cylinders for collision
•

Notes:
 
• Path generation can be made parallel: simply try each type of path simultaneously, then pick the best one that worked
