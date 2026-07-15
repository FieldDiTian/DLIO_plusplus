#ifndef GICP_PLUSPLUS_RTK_GATE_HPP
#define GICP_PLUSPLUS_RTK_GATE_HPP

#include <cmath>

namespace gicp_plusplus {

// RTK covariance-quality gate primitives (P1 FIX 2026-07-14).
//
// A variance component qualifies only when it is FINITE, NONNEGATIVE, and at
// most the configured limit. The former plain `<= threshold` comparison
// accepted the finite negative sentinel (-1 = "covariance not populated") as
// RTK-quality, letting unknown-quality /gps_p1/filtered_odom samples drive
// the INS heading prior, RTK bias calibration, and the GICP-vs-GT
// cross-check. This matches the adapter's stricter
// /gps_p1/filtered_odom_rtk_fixed gate (finite, nonnegative, thresholded).
// NaN fails closed via the isfinite test.
inline bool rtkCovarianceComponentOk(double var, double max_var) {
  return std::isfinite(var) && var >= 0.0 && var <= max_var;
}

inline bool rtkPositionCovarianceOk(double cov_xx, double cov_yy, double cov_zz,
                                    double max_var_xy, double max_var_z) {
  return rtkCovarianceComponentOk(cov_xx, max_var_xy) &&
         rtkCovarianceComponentOk(cov_yy, max_var_xy) &&
         rtkCovarianceComponentOk(cov_zz, max_var_z);
}

}  // namespace gicp_plusplus

#endif  // GICP_PLUSPLUS_RTK_GATE_HPP
