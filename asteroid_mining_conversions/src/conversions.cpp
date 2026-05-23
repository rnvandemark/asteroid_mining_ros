#include <asteroid_mining_conversions/conversions.hpp>

#include <algorithm>

namespace asteroid_mining_conversions {

geometry_msgs::msg::Vector3 cnv(const Eigen::Vector3d& in)
{
    return geometry_msgs::msg::Vector3().set__x(in.x()).set__y(in.y()).set__z(in.z());
}
Eigen::Vector3d cnv(const geometry_msgs::msg::Vector3& in)
{
    return Eigen::Vector3d{in.x, in.y, in.z};
}

asteroid_mining_interfaces::msg::ThirdOrderKinematicsOneDof cnv(
    const double in_pos,
    const double in_vel,
    const double in_acc
)
{
    return asteroid_mining_interfaces::msg::ThirdOrderKinematicsOneDof()
        .set__position(in_pos)
        .set__velocity(in_vel)
        .set__acceleration(in_acc)
    ;
}
std::tuple<double, double, double> cnv(
    const asteroid_mining_interfaces::msg::ThirdOrderKinematicsOneDof& in
)
{
    return std::make_tuple(in.position, in.velocity, in.acceleration);
}

asteroid_mining_interfaces::msg::ThirdOrderKinematicsThreeDof cnv(
    const Eigen::Vector3d& in_pos,
    const Eigen::Vector3d& in_vel,
    const Eigen::Vector3d& in_acc
)
{
    return asteroid_mining_interfaces::msg::ThirdOrderKinematicsThreeDof()
        .set__position(cnv(in_pos))
        .set__velocity(cnv(in_vel))
        .set__acceleration(cnv(in_acc))
    ;
}
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> cnv(
    const asteroid_mining_interfaces::msg::ThirdOrderKinematicsThreeDof& in
)
{
    return std::make_tuple(cnv(in.position), cnv(in.velocity), cnv(in.acceleration));
}

asteroid_mining_interfaces::msg::Asteroid cnv(const asteroid_mining::AsteroidCarrier& in)
{
    return asteroid_mining_interfaces::msg::Asteroid()
        .set__alpha(in.get_alpha())
        .set__beta(in.get_beta())
        .set__gamma(in.get_gamma())
        .set__density(in.get_density())
        .set__angular_velocity(in.get_angular_velocity())
        .set__omega(in.get_omega())
        .set__rotation(in.get_rotation())
    ;
}
asteroid_mining::AsteroidCarrier cnv(const asteroid_mining_interfaces::msg::Asteroid& in)
{
    return asteroid_mining::AsteroidCarrier(
        in.alpha,
        in.beta,
        in.gamma,
        in.density,
        in.angular_velocity,
        in.omega,
        in.rotation
    );
}

asteroid_mining_interfaces::msg::ReleasedPayload cnv(const asteroid_mining::ReleasedPayloadCarrier& in)
{
    return asteroid_mining_interfaces::msg::ReleasedPayload()
        .set__active(in.is_active())
        .set__mass(in.get_mass())
        .set__kinematics(cnv(in.get_position(), in.get_velocity(), in.get_acceleration()))
    ;
}
asteroid_mining::ReleasedPayloadCarrier cnv(const asteroid_mining_interfaces::msg::ReleasedPayload& in)
{
    Eigen::Vector3d pos, vel, acc;
    std::tie(pos, vel, acc) = cnv(in.kinematics);
    return asteroid_mining::ReleasedPayloadCarrier(
        in.active,
        in.mass,
        pos,
        vel,
        acc
    );
}

asteroid_mining_interfaces::msg::Siphon cnv(const asteroid_mining::SiphonCarrier& in)
{
    std::vector<double> mass_positions(in.get_n());
    std::vector<geometry_msgs::msg::Vector3> mass_effective_forces(in.get_n());
    for (std::size_t i = 0; i < in.get_n(); i++)
    {
        mass_positions[i] = in.get_mass_position(i);
        mass_effective_forces[i] = cnv(in.get_mass_effective_force(i));
    }

    return asteroid_mining_interfaces::msg::Siphon()
        .set__n(in.get_n())
        .set__chain_length(in.get_chain_length())
        .set__bucket_mass(in.get_bucket_mass())
        .set__payload_mass(in.get_payload_mass())
        .set__lifting_side_mass(in.get_lifting_side_mass())
        .set__descending_side_mass(in.get_descending_side_mass())
        .set__cs_dry_mass(in.get_cs_dry_mass())
        .set__anchor_point_polar_angle(in.get_anchor_point_polar_angle())
        .set__anchor_point_polar_radius(in.get_anchor_point_polar_radius())
        .set__max_bottom_lifting_side_mass_position(in.get_max_bottom_lifting_side_mass_position())
        .set__siphon_angular_kinematics(cnv(
            in.get_siphon_angular_position(),
            in.get_siphon_angular_velocity(),
            in.get_siphon_angular_acceleration()
        ))
        .set__bottom_lifting_side_mass_kinematics(cnv(
            in.get_bottom_lifting_side_mass_position(),
            in.get_bottom_lifting_side_mass_velocity(),
            in.get_bottom_lifting_side_mass_acceleration()
        ))
        .set__cs_payload_mass(in.get_cs_payload_mass())
        .set__total_time_elapsed(in.get_total_time_elapsed())
        .set__time_mass_reached_cs_last(in.get_time_mass_reached_cs_last())
        .set__time_elapsed_last_mass_to_reach_cs(in.get_time_elapsed_last_mass_to_reach_cs())
        .set__last_siphon_angular_velocity_was_positive(in.get_last_siphon_angular_velocity_was_positive())
        .set__last_min_siphon_angular_position_reached(in.get_last_min_siphon_angular_position_reached())
        .set__last_max_siphon_angular_position_reached(in.get_last_max_siphon_angular_position_reached())
        .set__mass_positions(mass_positions)
        .set__mass_effective_forces(mass_effective_forces)
    ;
}
asteroid_mining::SiphonCarrier cnv(const asteroid_mining_interfaces::msg::Siphon& in)
{
    const auto [siphon_angular_pos, siphon_angular_vel, siphon_angular_acc] = cnv(in.siphon_angular_kinematics);
    const auto [bottom_lifting_side_mass_pos, bottom_lifting_side_mass_vel, bottom_lifting_side_mass_acc] = cnv(in.bottom_lifting_side_mass_kinematics);

    std::vector<Eigen::Vector3d> mass_effective_forces(in.n);
    std::transform(
        in.mass_effective_forces.cbegin(),
        in.mass_effective_forces.cend(),
        mass_effective_forces.begin(),
        [] (const geometry_msgs::msg::Vector3& msg) -> Eigen::Vector3d { return cnv(msg); }
    );

    return asteroid_mining::SiphonCarrier(
        in.n,
        in.chain_length,
        in.bucket_mass,
        in.payload_mass,
        in.lifting_side_mass,
        in.descending_side_mass,
        in.cs_dry_mass,
        in.anchor_point_polar_angle,
        in.anchor_point_polar_radius,
        in.max_bottom_lifting_side_mass_position,
        siphon_angular_pos,
        siphon_angular_vel,
        siphon_angular_acc,
        bottom_lifting_side_mass_pos,
        bottom_lifting_side_mass_vel,
        bottom_lifting_side_mass_acc,
        in.cs_payload_mass,
        in.total_time_elapsed,
        in.time_mass_reached_cs_last,
        in.time_elapsed_last_mass_to_reach_cs,
        in.last_siphon_angular_velocity_was_positive,
        in.last_min_siphon_angular_position_reached,
        in.last_max_siphon_angular_position_reached,
        in.mass_positions,
        mass_effective_forces
    );
}

}
