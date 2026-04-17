#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* 2-D pose: world-frame (x, y, theta). */
typedef struct { double x, y, theta; } CeresPose;

/* Relative-pose constraint between two submap indices.
   weight: sequential = 1.0, loop-closure = 3.0 */
typedef struct {
    int    from, to;
    double dx, dy, dtheta;
    double weightXY;
    double weightTheta;
} CeresEdge;

/* Optimise poses[] in-place.  poses[0] is held fixed as the world anchor. */
void ceres_optimize_poses(CeresPose* poses, int n_poses,
                          const CeresEdge* edges, int n_edges);

#ifdef __cplusplus
}
#endif
