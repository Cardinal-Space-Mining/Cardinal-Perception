/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

/***********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, SMRT-AIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#pragma once

#include <boost/format.hpp>

#include <nano_gicp/gicp/so3.hpp>
// #include <nano_gicp/lsq_registration.hpp>

namespace nano_gicp
{

template<typename PointTarget, typename PointSource>
LsqRegistration<PointTarget, PointSource>::LsqRegistration()
{
    this->reg_name_ = "LsqRegistration";
    max_iterations_ = 64;
    rotation_epsilon_ = 2e-3;
    transformation_epsilon_ = 5e-4;

    lsq_optimizer_type_ = LSQ_OPTIMIZER_TYPE::LevenbergMarquardt;
    lm_debug_print_ = false;
    lm_max_iterations_ = 10;
    lm_init_lambda_factor_ = 1e-9;
    lm_lambda_ = -1.0;

    final_hessian_.setIdentity();
}

template<typename PointTarget, typename PointSource>
LsqRegistration<PointTarget, PointSource>::~LsqRegistration()
{
}

template<typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setRotationEpsilon(double eps)
{
    rotation_epsilon_ = eps;
}

template<typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setInitialLambdaFactor(
    double init_lambda_factor)
{
    lm_init_lambda_factor_ = init_lambda_factor;
}

template<typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setDebugPrint(
    bool lm_debug_print)
{
    lm_debug_print_ = lm_debug_print;
}

template<typename PointTarget, typename PointSource>
const Eigen::Matrix<double, 6, 6>&
    LsqRegistration<PointTarget, PointSource>::getFinalHessian() const
{
    return final_hessian_;
}

template<typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::computeTransformation(
    PointCloudSource& output,
    const Matrix4& guess)
{
    Eigen::Isometry3d x0 = Eigen::Isometry3d(guess.template cast<double>());

    lm_lambda_ = -1.0;
    converged_ = false;

    if (lm_debug_print_)
    {
        std::cout << "********************************************\n"
                     "***************** optimize *****************\n"
                     "********************************************"
                  << std::endl;
    }

    for (int i = 0; i < max_iterations_ && !converged_; i++)
    {
        nr_iterations_ = i;

        Eigen::Isometry3d delta;
        if (!step_optimize(x0, delta))
        {
            std::cerr << "lm not converged!!" << std::endl;
            break;
        }

        converged_ = is_converged(delta);
    }

    final_transformation_ = x0.cast<float>().matrix();
    pcl::transformPointCloud(*input_, output, final_transformation_);
}

template<typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::is_converged(
    const Eigen::Isometry3d& delta) const
{
    // double accum = 0.0;
    Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = delta.translation();

    Eigen::Matrix3d r_delta = 1.0 / rotation_epsilon_ * R.array().abs();
    Eigen::Vector3d t_delta = 1.0 / transformation_epsilon_ * t.array().abs();

    return std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) < 1;
}

template<typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_optimize(
    Eigen::Isometry3d& x0,
    Eigen::Isometry3d& delta)
{
    switch (lsq_optimizer_type_)
    {
        case LSQ_OPTIMIZER_TYPE::LevenbergMarquardt:
            return step_lm(x0, delta);
        case LSQ_OPTIMIZER_TYPE::GaussNewton:
            return step_gn(x0, delta);
    }

    return step_lm(x0, delta);
}

template<typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_gn(
    Eigen::Isometry3d& x0,
    Eigen::Isometry3d& delta)
{
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> b;
    /*double y0 =*/linearize(x0, &H, &b);

    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H);
    Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

    delta.setIdentity();
    delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
    delta.translation() = d.tail<3>();

    x0 = delta * x0;
    final_hessian_ = H;

    return true;
}

template<typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_lm(
    Eigen::Isometry3d& x0,
    Eigen::Isometry3d& delta)
{
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> b;
    double y0 = linearize(x0, &H, &b);

    if (lm_lambda_ < 0.0)
    {
        lm_lambda_ =
            lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
    }

    double nu = 2.0;
    for (int i = 0; i < lm_max_iterations_; i++)
    {
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(
            H + lm_lambda_ * Eigen::Matrix<double, 6, 6>::Identity());
        Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

        delta.setIdentity();
        delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
        delta.translation() = d.tail<3>();

        Eigen::Isometry3d xi = delta * x0;
        double yi = compute_error(xi);
        double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b));

        if (lm_debug_print_)
        {
            if (i == 0)
            {
                std::cout
                    << boost::format(
                           "--- LM optimization ---\n%5s %15s %15s %15s %15s %15s %5s\n") %
                           "i" % "y0" % "yi" % "rho" % "lambda" % "|delta|" %
                           "dec";
            }
            char dec = rho > 0.0 ? 'x' : ' ';
            std::cout << boost::format("%5d %15g %15g %15g %15g %15g %5c") % i %
                             y0 % yi % rho % lm_lambda_ % d.norm() % dec
                      << std::endl;
        }

        if (rho < 0)
        {
            if (is_converged(delta))
            {
                return true;
            }

            lm_lambda_ = nu * lm_lambda_;
            nu = 2 * nu;
            continue;
        }

        x0 = xi;
        lm_lambda_ =
            lm_lambda_ * std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
        final_hessian_ = H;
        return true;
    }

    return false;
}

}  // namespace nano_gicp
