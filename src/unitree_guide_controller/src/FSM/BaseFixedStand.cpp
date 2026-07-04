//
// Created by biao on 24-9-10.
//

#include "unitree_guide_controller/FSM/BaseFixedStand.h"

#include <cmath>

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>

BaseFixedStand::BaseFixedStand(CtrlInterfaces& ctrl_interfaces, CtrlComponent& ctrl_component,
                               const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDSTAND, "fixed stand", ctrl_interfaces),
      kp_(kp), kd_(kd),
      estimator_(ctrl_component.estimator_),
      robot_model_(ctrl_component.robot_model_),
      balance_ctrl_(ctrl_component.balance_ctrl_),
      wave_generator_(ctrl_component.wave_generator_)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
    }

    // Same gains StateBalanceTest uses for holding a planted stance - reused
    // rather than guessed, since that state's whole point is exactly this
    // (QP force balance while all 4 feet stay in stance).
    Kp_p_ = Vec3(150, 150, 150).asDiagonal();
    Kd_p_ = Vec3(25, 25, 25).asDiagonal();
    kp_w_ = 200;
    Kd_w_ = Vec3(30, 30, 30).asDiagonal();
}

void BaseFixedStand::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().value();
    }
    for (int i = 0; i < 12; i++)
    {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    ctrl_interfaces_.control_inputs_.command = 0;
    holding_ = false;
}

void BaseFixedStand::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);

    if (phase < 0.99)
    {
        for (int i = 0; i < 12; i++)
        {
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
                phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
        }
        return;
    }

    if (!holding_)
    {
        holding_ = true;
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
        pcd_hold_ = estimator_->getPosition();
        rotation_hold_ = estimator_->getRotation();
        // Hand primary authority to the torque command below - low PD here
        // is only a light trim, matching StateBalanceTest's own values.
        for (int i = 0; i < 12; i++)
        {
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.8);
            std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.8);
        }
    }
    calcBalanceTorque();
}

void BaseFixedStand::calcBalanceTorque()
{
    const auto B2G_Rotation = estimator_->getRotation();
    const RotMat G2B_Rotation = B2G_Rotation.transpose();

    const Vec3 pose_body = estimator_->getPosition();
    const Vec3 vel_body = estimator_->getVelocity();

    // Expected body acceleration/angular acceleration to return to the held
    // pose - same formulation as StateBalanceTest::calcTorque(), except the
    // target is fixed at whatever pose standing settled into (pcd_hold_/
    // rotation_hold_), not driven by joystick input.
    const Vec3 dd_pcd = Kp_p_ * (pcd_hold_ - pose_body) + Kd_p_ * (Vec3(0, 0, 0) - vel_body);
    const Vec3 d_wbd = kp_w_ * rotMatToExp(rotation_hold_ * G2B_Rotation) +
                        Kd_w_ * (Vec3(0, 0, 0) - estimator_->getGyroGlobal());

    const Vec34 pos_feet_2_body_global = estimator_->getFeetPos2Body();
    const Vec34 force_feet_global = -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_Rotation,
                                                         pos_feet_2_body_global, wave_generator_->contact_);
    const Vec34 force_feet_body = G2B_Rotation * force_feet_global;

    const std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++)
    {
        const KDL::JntArray torque = robot_model_->getTorque(force_feet_body.col(i), i);
        for (int j = 0; j < 3; j++)
        {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i * 3 + j].get().set_value(
                current_joints[i](j));
        }
    }
}

void BaseFixedStand::exit()
{
    percent_ = 0;
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName BaseFixedStand::checkChange()
{
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}
