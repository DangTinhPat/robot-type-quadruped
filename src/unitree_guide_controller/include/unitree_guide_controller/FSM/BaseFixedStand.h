//
// Created by biao on 24-9-10.
//
#pragma once

#include "FSMState.h"
#include <unitree_guide_controller/common/mathTypes.h>

class WaveGenerator;
class BalanceCtrl;
class QuadrupedRobot;
class Estimator;
struct CtrlComponent;

class BaseFixedStand : public FSMState
{
public:
    BaseFixedStand(CtrlInterfaces& ctrl_interfaces,
                   CtrlComponent& ctrl_component,
                   const std::vector<double>& target_pos,
                   double kp,
                   double kd);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

protected:
    double target_pos_[12] = {};
    double start_pos_[12] = {};

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;

private:
    // Once the tanh position interpolation above settles into the target
    // stand pose, holding it switches from open-loop joint-angle PD to the
    // same QP whole-body force balance StateBalanceTest uses: real IMU/leg-
    // kinematics feedback (Estimator) drives BalanceCtrl's force allocation,
    // so a sideways push is actively resisted instead of only being fought by
    // fixed joint-angle setpoints that have no idea the body tilted.
    void calcBalanceTorque();

    std::shared_ptr<Estimator>& estimator_;
    std::shared_ptr<QuadrupedRobot>& robot_model_;
    std::shared_ptr<BalanceCtrl>& balance_ctrl_;
    std::shared_ptr<WaveGenerator>& wave_generator_;

    bool holding_ = false;
    Vec3 pcd_hold_;
    RotMat rotation_hold_;
    Mat3 Kp_p_, Kd_p_, Kd_w_;
    double kp_w_{};
};
