//
// Created by guan on 2025/11/7.
//
#include "app_custom_joint.h"

#include "bsp_time.h"
#include "ctrl_forward_feed.h"
#include "ctrl_motor_base_pid.h"

void Joint::joint_init(){
    if(motor_ctrl_) {
        motor_ctrl_->init();
        motor_ctrl_->add_controller(std::make_unique <Controller::MotorBasePID> (
            Controller::MotorBasePID::PID_SPEED | Controller::MotorBasePID::PID_ANGLE,
            std::move(speed_pid_),
            std::move(angle_pid_),
            true
        ));
        motor_ctrl_->add_controller([&](const MotorController *ptr) -> float {
            return static_cast<float>(t_tor * GM6020_TOR_TO_COUNT);;
        }, std::make_unique <Controller::ForwardFeed> (-16384, 16384));
        motor_ctrl_->use_extend_angle = true;
        motor_ctrl_->use_degree_angle = true;
        motor_ctrl_->encoder_zero = zero_position_;
        enable_flag_ = true;
    }
}

void Joint::joint_update(bool use_tar, double target, double tor_t) {
    if(motor_ctrl_) {
        use_tar_ = use_tar;

        t_tor = tor_t;
        if(feedforward_func_ != nullptr) t_tor += feedforward_func_(angle_*M_PI/180);
        target_ = target;
        torque_ = motor_ctrl_->output / GM6020_TOR_TO_COUNT;
        angle_ = motor_ctrl_->angle;
        error_code_ = joint_limit(t_tor);
        if(error_code_&APP_JOINT_ERROR_ANGLE_LIMIT) enable_flag_ = false, t_tor = 0;
        if(error_code_&APP_JOINT_ERROR_TORQUE_UP_LIMIT) t_tor = max_torque_;
        if(error_code_&APP_JOINT_ERROR_TORQUE_DOWN_LIMIT) t_tor = -max_torque_;

        if(enable_flag_) {
            if(use_tar_) {
                motor_ctrl_->update(target);
            }else {
                motor_ctrl_->clear();
                motor_ctrl_->update(motor_ctrl_->angle);
            }
        }else {
            motor_ctrl_->relax();
        }

        lst_online_time = bsp_time_get_us() - online_time;
        online_time = bsp_time_get_us();
    }
}
