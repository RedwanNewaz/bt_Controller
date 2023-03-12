//
// Created by redwan on 3/11/23.
//

#ifndef CREATE3_CONTROLLER_EKF_H
#define CREATE3_CONTROLLER_EKF_H
#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/twist.hpp>

class EKF {
public:
    explicit EKF(const double dt);
    void update(const tf2::Transform& obs, const geometry_msgs::msg::Twist& cmd,  tf2::Transform& res);
private:
    [[nodiscard]] Eigen::Vector4f motion_model(const Eigen::Vector4f &x, const Eigen::Vector2f &u) const;
    [[nodiscard]] Eigen::Matrix4f jacobF(const Eigen::Vector4f& x, const Eigen::Vector2f& u) const;
    [[nodiscard]] Eigen::Vector2f observation_model(const Eigen::Vector4f& x) const ;
    [[nodiscard]] Eigen::Matrix<float, 2, 4> jacobH()const;
    void ekf_estimation(const Eigen::Vector2f& z, const Eigen::Vector2f& u);

    Eigen::Matrix4f Q;
    Eigen::Matrix2f R;
    Eigen::Vector4f xEst;
    Eigen::Matrix4f PEst;
    const double DT;
    bool initialized;

};


#endif //CREATE3_CONTROLLER_EKF_H
