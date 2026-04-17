#include "ceres_wrapper.h"

#ifndef HAVE_CERES
// Ceres not available — provide a no-op stub so the package links cleanly.
// The real optimisation is handled by optimizer_fallback.go (Go side).
extern "C"
void ceres_optimize_poses(CeresPose*, int, const CeresEdge*, int) {}
#else

#include <ceres/ceres.h>
#include <cmath>

// ---------------------------------------------------------------------------
// Cost functor: edge residual for one pose-graph constraint.
//
// Residuals are scaled by sqrt(weight) so that Ceres' sum-of-squares
// objective equals sum_edges  weight * ||r||^2  before the Huber kernel.
// HuberLoss(0.10 m) additionally down-weights outlier edges.
// ---------------------------------------------------------------------------
struct PoseGraphEdge {
    double dx, dy, dtheta, sqrt_wXY, sqrt_wTheta;

    PoseGraphEdge(double dx, double dy, double dtheta,
                  double weight_xy, double weight_theta)
        : dx(dx), dy(dy), dtheta(dtheta),
          sqrt_wXY(std::sqrt(weight_xy)), sqrt_wTheta(std::sqrt(weight_theta)) {}

    template <typename T>
    bool operator()(const T* const p_from,
                    const T* const p_to,
                    T* residuals) const {
        
        T cos_theta = ceres::cos(p_from[2]);
        T sin_theta = ceres::sin(p_from[2]);

        T expected_dx = cos_theta * T(dx) - sin_theta * T(dy);
        T expected_dy = sin_theta * T(dx) + cos_theta * T(dy);

        // Translation residuals
        residuals[0] = T(sqrt_wXY) * (p_to[0] - p_from[0] - expected_dx);
        residuals[1] = T(sqrt_wXY) * (p_to[1] - p_from[1] - expected_dy);

        // Rotation residual — keep in [-π, π] via atan2(sin, cos)
        T delta = p_to[2] - p_from[2] - T(dtheta);
        residuals[2] = T(sqrt_wTheta) * ceres::atan2(ceres::sin(delta), ceres::cos(delta));
        return true;
    }

    static ceres::CostFunction* Create(double dx, double dy, double dtheta,
                                       double weight_xy, double weight_theta) {
        // 3 residuals, two parameter blocks of size 3 each.
        return new ceres::AutoDiffCostFunction<PoseGraphEdge, 3, 3, 3>(
            new PoseGraphEdge(dx, dy, dtheta, weight_xy, weight_theta));
    }
};

// ---------------------------------------------------------------------------

extern "C"
void ceres_optimize_poses(CeresPose* poses, int n_poses,
                          const CeresEdge* edges, int n_edges) {
    if (n_poses < 2 || n_edges == 0) return;

    ceres::Problem problem;

    for (int i = 0; i < n_edges; i++) {
        int f = edges[i].from, t = edges[i].to;
        if (f < 0 || t >= n_poses || f >= t) continue;

        double* p_from = reinterpret_cast<double*>(&poses[f]);
        double* p_to   = reinterpret_cast<double*>(&poses[t]);

        ceres::CostFunction* cost = PoseGraphEdge::Create(
            edges[i].dx, edges[i].dy, edges[i].dtheta, edges[i].weightXY, edges[i].weightTheta);

        // Huber kernel: quadratic below 0.05 m, linear above.
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.10),
                                 p_from, p_to);
    }

    // Submap 0 is the world anchor — never moved.
    problem.SetParameterBlockConstant(reinterpret_cast<double*>(&poses[0]));

    ceres::Solver::Options opts;
    opts.max_num_iterations          = 1000;
    opts.linear_solver_type          = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.minimizer_progress_to_stdout = false;
    opts.logging_type                = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
}

#endif // HAVE_CERES
