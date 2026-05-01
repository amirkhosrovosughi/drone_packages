#ifndef SLAM__COMMON__DEF_SLAM_OPTIMIZATION_HPP_
#define SLAM__COMMON__DEF_SLAM_OPTIMIZATION_HPP_

#include <string>

/**
 * @brief Refinement strategy for pose optimization.
 * Allows future flexibility between different optimization approaches.
 */
enum class RefinementStrategy {
  GaussNewton,    // Iterative Gauss-Newton for accuracy
  ClosedForm      // Single-step closed-form for speed
};

/**
 * @brief Configuration for optimization/refinement operations.
 */
struct OptimizationConfig {
  int maxIterations = 5;           // Max iterations for Gauss-Newton
  int maxSolveTimeMs = 100;        // Max time budget for solve (ms)
  double convergeThreshold = 1e-4; // Convergence criterion (pose delta norm)
  RefinementStrategy strategy = RefinementStrategy::GaussNewton;

  OptimizationConfig() = default;
};

/**
 * @brief Result from optimization/refinement operations.
 */
struct OptimizationResult {
  bool success = false;                   // Whether optimization succeeded
  int solveTimeMs = 0;                    // Actual time taken (ms)
  double finalError = 0.0;                // Final residual error
  double poseJumpMeters = 0.0;            // Active-pose jump distance after optimize
  int numPosesRefined = 0;                // Number of poses updated
  int numIterations = 0;                  // Actual iterations run
  std::string failureReason = "";         // Reason if success=false

  OptimizationResult() = default;
};

#endif  // SLAM__COMMON__DEF_SLAM_OPTIMIZATION_HPP_
