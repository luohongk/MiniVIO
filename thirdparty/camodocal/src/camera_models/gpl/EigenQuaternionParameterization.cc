#include "camodocal/gpl/EigenQuaternionParameterization.h"

#include <cmath>

namespace camodocal
{

bool
EigenQuaternionManifold::Plus(const double* x,
                              const double* delta,
                              double* x_plus_delta) const
{
    const double norm_delta =
        sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    if (norm_delta > 0.0)
    {
        const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
        double q_delta[4];
        q_delta[0] = sin_delta_by_delta * delta[0];
        q_delta[1] = sin_delta_by_delta * delta[1];
        q_delta[2] = sin_delta_by_delta * delta[2];
        q_delta[3] = cos(norm_delta);
        EigenQuaternionProduct(q_delta, x, x_plus_delta);
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            x_plus_delta[i] = x[i];
        }
    }
    return true;
}

bool
EigenQuaternionManifold::PlusJacobian(const double* x,
                                      double* jacobian) const
{
    jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];  // NOLINT
    jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];  // NOLINT
    jacobian[6] =  x[1]; jacobian[7] = -x[0]; jacobian[8] =  x[3];  // NOLINT
    jacobian[9] = -x[0]; jacobian[10]  = -x[1]; jacobian[11]  = -x[2];  // NOLINT
    return true;
}

bool
EigenQuaternionManifold::Minus(const double* y,
                               const double* x,
                               double* y_minus_x) const
{
    // Compute q_diff = y * x^{-1}
    // For unit quaternion, inverse is conjugate: x^{-1} = [-x0, -x1, -x2, x3]
    double x_inv[4] = {-x[0], -x[1], -x[2], x[3]};
    double q_diff[4];
    EigenQuaternionProduct(y, x_inv, q_diff);

    // Convert quaternion difference to angle-axis
    const double sin_half_angle = sqrt(q_diff[0] * q_diff[0] +
                                       q_diff[1] * q_diff[1] +
                                       q_diff[2] * q_diff[2]);
    if (sin_half_angle > 0.0)
    {
        const double cos_half_angle = q_diff[3];
        double two_theta;
        if (cos_half_angle >= 0.0)
        {
            two_theta = 2.0 * atan2(sin_half_angle, cos_half_angle);
        }
        else
        {
            two_theta = 2.0 * atan2(-sin_half_angle, -cos_half_angle);
        }
        const double k = two_theta / sin_half_angle;
        y_minus_x[0] = q_diff[0] * k;
        y_minus_x[1] = q_diff[1] * k;
        y_minus_x[2] = q_diff[2] * k;
    }
    else
    {
        y_minus_x[0] = 0.0;
        y_minus_x[1] = 0.0;
        y_minus_x[2] = 0.0;
    }
    return true;
}

bool
EigenQuaternionManifold::MinusJacobian(const double* x,
                                       double* jacobian) const
{
    // MinusJacobian at identity is the same as PlusJacobian transposed
    // For simplicity, use the same form as PlusJacobian
    jacobian[0] =  x[3]; jacobian[1]  = -x[2]; jacobian[2]  =  x[1]; jacobian[3]  = -x[0];  // NOLINT
    jacobian[4] =  x[2]; jacobian[5]  =  x[3]; jacobian[6]  = -x[0]; jacobian[7]  = -x[1];  // NOLINT
    jacobian[8] = -x[1]; jacobian[9]  =  x[0]; jacobian[10] =  x[3]; jacobian[11] = -x[2];  // NOLINT
    return true;
}

}
