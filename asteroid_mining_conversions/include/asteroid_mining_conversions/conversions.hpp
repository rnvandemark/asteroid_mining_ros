#pragma once

#include <asteroid_mining/asteroid/carrier.hpp>
#include <asteroid_mining/released_payload/carrier.hpp>
#include <asteroid_mining/siphon/carrier.hpp>
#include <asteroid_mining_interfaces/msg/asteroid.hpp>
#include <asteroid_mining_interfaces/msg/released_payload.hpp>
#include <asteroid_mining_interfaces/msg/siphon.hpp>

#include <tuple>

namespace asteroid_mining_conversions {

geometry_msgs::msg::Vector3 cnv(const Eigen::Vector3d& in);
Eigen::Vector3d cnv(const geometry_msgs::msg::Vector3& in);

asteroid_mining_interfaces::msg::ThirdOrderKinematicsOneDof cnv(
    const double in_pos,
    const double in_vel,
    const double in_acc
);
std::tuple<double, double, double> cnv(
    const asteroid_mining_interfaces::msg::ThirdOrderKinematicsOneDof& in
);

asteroid_mining_interfaces::msg::ThirdOrderKinematicsThreeDof cnv(
    const Eigen::Vector3d& in_pos,
    const Eigen::Vector3d& in_vel,
    const Eigen::Vector3d& in_acc
);
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> cnv(
    const asteroid_mining_interfaces::msg::ThirdOrderKinematicsThreeDof& in
);

asteroid_mining_interfaces::msg::Asteroid cnv(const asteroid_mining::AsteroidCarrier& in);
asteroid_mining::AsteroidCarrier cnv(const asteroid_mining_interfaces::msg::Asteroid& in);

asteroid_mining_interfaces::msg::ReleasedPayload cnv(const asteroid_mining::ReleasedPayloadCarrier& in);
asteroid_mining::ReleasedPayloadCarrier cnv(const asteroid_mining_interfaces::msg::ReleasedPayload& in);

asteroid_mining_interfaces::msg::Siphon cnv(const asteroid_mining::SiphonCarrier& in);
asteroid_mining::SiphonCarrier cnv(const asteroid_mining_interfaces::msg::Siphon& in);

}
