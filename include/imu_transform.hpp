#pragma once

#include <tf2_ros/buffer_interface.h>
#include <tf2/convert.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace tf2
{
/**********/
/** IMU  **/
/**********/

/**
* method to extract timestamp from object
*/
template <>
inline
tf2::TimePoint getTimestamp(const sensor_msgs::msg::Imu& p) {return tf2_ros::fromMsg(p.header.stamp);}

/**
* method to extract frame id from object
*/
template <>
inline
std::string getFrameId(const sensor_msgs::msg::Imu &p) {return p.header.frame_id;}


/**
* Transforms a covariance array from one frame to another
*/
inline
void transformCovariance(const std::array<double, 9>& in, std::array<double, 9>& out, Eigen::Quaternion<double> r){

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_out(out.data());
    cov_out = r * cov_in * r.inverse();

}

/**
* Transforms sensor_msgs::Imu data from one frame to another
*/
template <>
inline
void doTransform(const sensor_msgs::msg::Imu &imu_in, sensor_msgs::msg::Imu &imu_out, const geometry_msgs::msg::TransformStamped& t_in)
{

    imu_out.header = t_in.header;

    // Discard translation, only use orientation for IMU transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d vel = t * Eigen::Vector3d(
        imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

    imu_out.angular_velocity.x = vel.x();
    imu_out.angular_velocity.y = vel.y();
    imu_out.angular_velocity.z = vel.z();

    transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

    Eigen::Vector3d accel = t * Eigen::Vector3d(
        imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);


    imu_out.linear_acceleration.x = accel.x();
    imu_out.linear_acceleration.y = accel.y();
    imu_out.linear_acceleration.z = accel.z();

    transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

    // Orientation expresses attitude of the new frame_id in a fixed world frame. This is why the transform here applies
    // in the opposite direction.
    Eigen::Quaternion<double> orientation = Eigen::Quaternion<double>(
        imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) * r.inverse();

    imu_out.orientation.w = orientation.w();
    imu_out.orientation.x = orientation.x();
    imu_out.orientation.y = orientation.y();
    imu_out.orientation.z = orientation.z();

    // Orientation is measured relative to the fixed world frame, so it doesn't change when applying a static
    // transform to the sensor frame.
    imu_out.orientation_covariance = imu_in.orientation_covariance;

}

inline
sensor_msgs::msg::Imu toMsg(const sensor_msgs::msg::Imu &in)
{
    return in;
}

inline
void fromMsg(const sensor_msgs::msg::Imu &msg, sensor_msgs::msg::Imu &out)
{
    out = msg;
}

/*********************/
/** Magnetic Field  **/
/*********************/

/**
* method to extract timestamp from object
*/
template <>
inline
tf2::TimePoint getTimestamp(const sensor_msgs::msg::MagneticField& p) {return tf2_ros::fromMsg(p.header.stamp);}

/**
* method to extract frame id from object
*/
template <>
inline
std::string getFrameId(const sensor_msgs::msg::MagneticField &p) {return p.header.frame_id;}

/**
* Transforms sensor_msgs::MagneticField data from one frame to another
*/
template <>
inline
void doTransform(const sensor_msgs::msg::MagneticField &mag_in, sensor_msgs::msg::MagneticField &mag_out, const geometry_msgs::msg::TransformStamped& t_in)
{

    mag_out.header = t_in.header;

    // Discard translation, only use orientation for Magnetic Field transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d mag = t * Eigen::Vector3d(
        mag_in.magnetic_field.x, mag_in.magnetic_field.y, mag_in.magnetic_field.z);

    mag_out.magnetic_field.x = mag.x();
    mag_out.magnetic_field.y = mag.y();
    mag_out.magnetic_field.z = mag.z();

    transformCovariance(mag_in.magnetic_field_covariance, mag_out.magnetic_field_covariance, r);

}

inline
sensor_msgs::msg::MagneticField toMsg(const sensor_msgs::msg::MagneticField &in)
{
    return in;
}

inline
void fromMsg(const sensor_msgs::msg::MagneticField &msg, sensor_msgs::msg::MagneticField &out)
{
    out = msg;
}

}
