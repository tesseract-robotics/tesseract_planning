# SimplePlanner

The SimplePlanner is a simple path interpolator. It is used to generate initial trajectories with a minimum number of waypoints needed for e.g. Trajopt.

## Profiles

Fixed size: fixed number of waypoints between initial waypoints.
LVS: LVS number of waypoints between initial waypoints.
Assign: No interpolation, but simple replication of (base/prev?) waypoints. This will allow Trajopt to evolve organically from the start to the goal.

SimplePlannerFixedSizeAssignNoIKMoveProfile
SimplePlannerFixedSizeAssignMoveProfile
SimplePlannerFixedSizeNoIKMoveProfile does not exist.
SimplePlannerFixedSizeMoveProfile
SimplePlannerLVSAssignNoIKMoveProfile
SimplePlannerLVSAssignMoveProfile
SimplePlannerLVSNoIKMoveProfile
SimplePlannerLVSMoveProfile
