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

#ifndef NANO_GICP_NANO_GICP_IMPL_HPP
#define NANO_GICP_NANO_GICP_IMPL_HPP

// #include <nano_gicp/nano_gicp.hpp>
#include <nano_gicp/gicp/so3.hpp>

namespace nano_gicp
{

template<typename PointSource, typename PointTarget>
NanoGICP<PointSource, PointTarget>::NanoGICP()
{
#ifdef _OPENMP
    num_threads_ = omp_get_max_threads();
#else
    num_threads_ = 1;
#endif

    k_correspondences_ = 20;
    reg_name_ = "NanoGICP";
    corr_dist_threshold_ = std::numeric_limits<float>::max();

    regularization_method_ = RegularizationMethod::PLANE;
    source_kdtree_.reset(new nanoflann::KdTreeFLANN<PointSource>);
    target_kdtree_.reset(new nanoflann::KdTreeFLANN<PointTarget>);
}

template<typename PointSource, typename PointTarget>
NanoGICP<PointSource, PointTarget>::~NanoGICP()
{
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setNumThreads(int n)
{
    num_threads_ = n;

#ifdef _OPENMP
    if (n == 0)
    {
        num_threads_ = omp_get_max_threads();
    }
#endif
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setCorrespondenceRandomness(int k)
{
    k_correspondences_ = k;
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setRegularizationMethod(
    RegularizationMethod method)
{
    regularization_method_ = method;
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::swapSourceAndTarget()
{
    input_.swap(target_);
    source_kdtree_.swap(target_kdtree_);
    source_covs_.swap(target_covs_);

    correspondences_.clear();
    sq_distances_.clear();
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::clearSource()
{
    input_.reset();
    source_covs_.clear();
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::clearTarget()
{
    target_.reset();
    target_covs_.clear();
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::registerInputSource(
    const PointCloudSourceConstPtr& cloud)
{
    if (input_ == cloud)
    {
        return;
    }
    pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setInputSource(
    const PointCloudSourceConstPtr& cloud)
{
    if (input_ == cloud)
    {
        return;
    }

    pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
    source_kdtree_->setInputCloud(cloud);
    source_covs_.clear();
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setInputTarget(
    const PointCloudTargetConstPtr& cloud)
{
    if (target_ == cloud)
    {
        return;
    }
    pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
    target_kdtree_->setInputCloud(cloud);
    target_covs_.clear();
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setSourceCovariances(
    const std::vector<
        Eigen::Matrix4d,
        Eigen::aligned_allocator<Eigen::Matrix4d>>& covs)
{
    source_covs_ = covs;
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::setTargetCovariances(
    const std::vector<
        Eigen::Matrix4d,
        Eigen::aligned_allocator<Eigen::Matrix4d>>& covs)
{
    target_covs_ = covs;
}

template<typename PointSource, typename PointTarget>
bool NanoGICP<PointSource, PointTarget>::calculateSourceCovariances()
{
    return calculate_covariances(input_, *source_kdtree_, source_covs_);
}

template<typename PointSource, typename PointTarget>
bool NanoGICP<PointSource, PointTarget>::calculateTargetCovariances()
{
    return calculate_covariances(target_, *target_kdtree_, target_covs_);
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::computeTransformation(
    PointCloudSource& output,
    const Matrix4& guess)
{
    if (source_covs_.size() != input_->size())
    {
        calculateSourceCovariances();
    }
    if (target_covs_.size() != target_->size())
    {
        calculateTargetCovariances();
    }

    LsqRegistration<PointSource, PointTarget>::computeTransformation(
        output,
        guess);
}

template<typename PointSource, typename PointTarget>
void NanoGICP<PointSource, PointTarget>::update_correspondences(
    const Eigen::Isometry3d& trans)
{
    assert(source_covs_.size() == input_->size());
    assert(target_covs_.size() == target_->size());

    Eigen::Isometry3f trans_f = trans.cast<float>();

    correspondences_.resize(input_->size());
    sq_distances_.resize(input_->size());
    mahalanobis_.resize(input_->size());

    std::vector<int> k_indices(1);
    std::vector<float> k_sq_dists(1);

#pragma omp parallel for num_threads(num_threads_)          \
    firstprivate(k_indices, k_sq_dists) schedule(guided, 8)
    for (size_t i = 0; i < input_->size(); i++)
    {
        PointTarget pt;
        pt.getVector4fMap() = trans_f * input_->at(i).getVector4fMap();

        target_kdtree_->nearestKSearch(pt, 1, k_indices, k_sq_dists);

        sq_distances_[i] = k_sq_dists[0];
        correspondences_[i] =
            k_sq_dists[0] < corr_dist_threshold_ * corr_dist_threshold_
                ? k_indices[0]
                : -1;

        if (correspondences_[i] < 0)
        {
            continue;
        }

        const int target_index = correspondences_[i];
        const auto& cov_A = source_covs_[i];
        const auto& cov_B = target_covs_[target_index];

        Eigen::Matrix4d RCR =
            cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
        RCR(3, 3) = 1.0;

        mahalanobis_[i] = RCR.inverse();
        mahalanobis_[i](3, 3) = 0.0f;
    }
}

template<typename PointSource, typename PointTarget>
double NanoGICP<PointSource, PointTarget>::linearize(
    const Eigen::Isometry3d& trans,
    Eigen::Matrix<double, 6, 6>* H,
    Eigen::Matrix<double, 6, 1>* b)
{
    update_correspondences(trans);

    double sum_errors = 0.0;
    std::vector<
        Eigen::Matrix<double, 6, 6>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>>
        Hs(num_threads_);
    std::vector<
        Eigen::Matrix<double, 6, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>
        bs(num_threads_);
    for (int i = 0; i < num_threads_; i++)
    {
        Hs[i].setZero();
        bs[i].setZero();
    }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) \
    schedule(guided, 8)
    for (size_t i = 0; i < input_->size(); i++)
    {
        int target_index = correspondences_[i];
        if (target_index < 0)
        {
            continue;
        }

        const Eigen::Vector4d mean_A =
            input_->at(i).getVector4fMap().template cast<double>();
        // const auto & cov_A = source_covs_[i];

        const Eigen::Vector4d mean_B =
            target_->at(target_index).getVector4fMap().template cast<double>();
        // const auto & cov_B = target_covs_[target_index];

        const Eigen::Vector4d transed_mean_A = trans * mean_A;
        const Eigen::Vector4d error = mean_B - transed_mean_A;

        sum_errors += error.transpose() * mahalanobis_[i] * error;

        if (H == nullptr || b == nullptr)
        {
            continue;
        }

        Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
        dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
        dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

        Eigen::Matrix<double, 6, 6> Hi =
            jlossexp.transpose() * mahalanobis_[i] * jlossexp;
        Eigen::Matrix<double, 6, 1> bi =
            jlossexp.transpose() * mahalanobis_[i] * error;

        Hs[omp_get_thread_num()] += Hi;
        bs[omp_get_thread_num()] += bi;
    }

    if (H && b)
    {
        H->setZero();
        b->setZero();
        for (int i = 0; i < num_threads_; i++)
        {
            (*H) += Hs[i];
            (*b) += bs[i];
        }
    }

    return sum_errors;
}

template<typename PointSource, typename PointTarget>
double NanoGICP<PointSource, PointTarget>::compute_error(
    const Eigen::Isometry3d& trans)
{
    double sum_errors = 0.0;

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) \
    schedule(guided, 8)
    for (size_t i = 0; i < input_->size(); i++)
    {
        int target_index = correspondences_[i];
        if (target_index < 0)
        {
            continue;
        }

        const Eigen::Vector4d mean_A =
            input_->at(i).getVector4fMap().template cast<double>();
        // const auto & cov_A = source_covs_[i];

        const Eigen::Vector4d mean_B =
            target_->at(target_index).getVector4fMap().template cast<double>();
        // const auto & cov_B = target_covs_[target_index];

        const Eigen::Vector4d transed_mean_A = trans * mean_A;
        const Eigen::Vector4d error = mean_B - transed_mean_A;

        sum_errors += error.transpose() * mahalanobis_[i] * error;
    }

    return sum_errors;
}

template<typename PointSource, typename PointTarget>
template<typename PointT>
bool NanoGICP<PointSource, PointTarget>::calculate_covariances(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    nanoflann::KdTreeFLANN<PointT>& kdtree,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>&
        covariances)
{
    if (kdtree.getInputCloud() != cloud)
    {
        kdtree.setInputCloud(cloud);
    }
    covariances.resize(cloud->size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
    for (size_t i = 0; i < cloud->size(); i++)
    {
        std::vector<int> k_indices;
        std::vector<float> k_sq_distances;
        kdtree.nearestKSearch(
            cloud->at(i),
            k_correspondences_,
            k_indices,
            k_sq_distances);

        Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
        for (size_t j = 0; j < k_indices.size(); j++)
        {
            neighbors.col(j) = cloud->at(k_indices[j])
                                   .getVector4fMap()
                                   .template cast<double>();
        }

        neighbors.colwise() -= neighbors.rowwise().mean().eval();
        Eigen::Matrix4d cov =
            neighbors * neighbors.transpose() / k_correspondences_;

        if (regularization_method_ == RegularizationMethod::NONE)
        {
            covariances[i] = cov;
        }
        else if (regularization_method_ == RegularizationMethod::FROBENIUS)
        {
            double lambda = 1e-3;
            Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() +
                                lambda * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d C_inv = C.inverse();
            covariances[i].setZero();
            covariances[i].template block<3, 3>(0, 0) =
                (C_inv / C_inv.norm()).inverse();
        }
        else
        {
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                cov.block<3, 3>(0, 0),
                Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d values;

            switch (regularization_method_)
            {
                default:
                    throw std::runtime_error("Here must not be reached");
                case RegularizationMethod::PLANE:
                    values = Eigen::Vector3d(1, 1, 1e-3);
                    break;
                case RegularizationMethod::MIN_EIG:
                    values = svd.singularValues().array().max(1e-3);
                    break;
                case RegularizationMethod::NORMALIZED_MIN_EIG:
                    values =
                        svd.singularValues() / svd.singularValues().maxCoeff();
                    values = values.array().max(1e-3);
                    break;
            }

            covariances[i].setZero();
            covariances[i].template block<3, 3>(0, 0) =
                svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
        }
    }

    return true;
}

}  // namespace nano_gicp

#endif
