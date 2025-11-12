//
// Created by guan on 2025/11/7.
//

#pragma once
#include "app_motor.h"

#include "ctrl_pid.h"

//0.0152 j2 杆长
//0.524kg 质量
#define SERIES_JOINT2_L 0.0152      //m
#define SERIES_MASS 0.524           //kg
#define G           9.80665

#define GM6020_Kt 0.741
#define GM6020_CURRENT_TO_COUNT (3.0/16384.0)
#define GM6020_TOR_TO_COUNT ((1/GM6020_Kt)/GM6020_CURRENT_TO_COUNT)

#define APP_JOINT_ERROR_ANGLE_LIMIT         0b0001  // 角度超限
#define APP_JOINT_ERROR_TORQUE_UP_LIMIT     0b0010  // 力矩上限
#define APP_JOINT_ERROR_TORQUE_DOWN_LIMIT   0b0100  // 力矩下限

class Joint {
    //采用角度制
public:
    Joint() = default;
    explicit Joint(std::unique_ptr<MotorController> motor_ctrl, float zero_position, float max_angle, float min_angle, float max_torque,
        std::unique_ptr<Controller::PID> speed, std::unique_ptr<Controller::PID> angle)
        : motor_ctrl_(std::move(motor_ctrl)), zero_position_(zero_position), max_angle_(max_angle), min_angle_(min_angle), max_torque_(max_torque),
        speed_pid_(std::move(speed)), angle_pid_(std::move(angle)) {}

    Joint(std::unique_ptr<MotorController> motor_ctrl, float zero_position, float max_angle, float min_angle, float max_torque,
        std::unique_ptr<Controller::PID> speed, std::unique_ptr<Controller::PID> angle, std::function<float(float)> feedforward_func)
        : motor_ctrl_(std::move(motor_ctrl)), zero_position_(zero_position), max_angle_(max_angle), min_angle_(min_angle), max_torque_(max_torque),
        speed_pid_(std::move(speed)), angle_pid_(std::move(angle)), feedforward_func_(std::move(feedforward_func)) {}

    void joint_init();
    void joint_update(bool use_tar, double target, double tor_t);
    // void tor_ctrl(float t_tor);
    void joint_reset() {
        enable_flag_ = true;
        error_code_ = 0;
        motor_ctrl_->activate();
    }
    void joint_relax() const {
        motor_ctrl_->relax();
    }

    double t_tor = 0;
    std::unique_ptr<MotorController> motor_ctrl_;

    bool enable_flag_ = false;
    double torque_ = 0, target_ = 0;
    float angle_ = 0;
    uint64_t online_time = 0, lst_online_time = 0;
private:
    [[nodiscard]] uint8_t joint_limit(double temp_tor) const {
        uint8_t flag = 0;
        temp_tor>max_torque_?flag+=0b0010:(temp_tor<-max_torque_?flag+=0b0100:0);
        angle_<=max_angle_?(angle_>=min_angle_?0:flag+=0b0001):flag+=0b0001;
        return flag;
    }

    float zero_position_;
    float max_angle_, min_angle_, max_torque_;

    // double t_tor = 0;
    uint8_t error_code_ = 0;
    bool use_tar_ = false;

    // std::unique_ptr<MotorController> motor_ctrl_;

    std::unique_ptr<Controller::PID> speed_pid_;        //初始化后会变成nullptr
    std::unique_ptr<Controller::PID> angle_pid_;        //初始化后会变成nullptr

    std::function<float(float)> feedforward_func_ = nullptr;
};