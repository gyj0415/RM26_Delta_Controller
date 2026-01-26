//
// Created by guan on 2025/11/5.
//

#include "app_custom.h"
#include "app_sys.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "dev_motor_dm.h"
// #include "app_custom_series.h"
#include "app_custom_joint.h"
#include "app_msg.h"
#include "ctrl_motor_base_pid.h"

#include "app_sys.h"
#include "dev_motor_dji.h"
#include "sys_task.h"

#ifdef COMPILE_CUSTOM

using namespace Motor;
using namespace Controller;

DMMotor DM_Motor1("Motor1",DMMotor::J4310,{
    .slave_id = 0x01,
    .master_id = 0x11,
    .port = E_CAN2,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
DMMotor DM_Motor2("Motor2",DMMotor::J4310,{
    .slave_id = 0x02,
    .master_id = 0x12,
    .port = E_CAN2,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
DMMotor DM_Motor3("Motor3",DMMotor::J4310,{
    .slave_id = 0x03,
    .master_id = 0x13,
    .port = E_CAN2,
    .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});

Joint joint1(
    std::make_unique<MotorController>(std::make_unique <DJIMotor> (
        "joint1",
        DJIMotor::GM6020,
        (DJIMotor::Param) { 0x01, E_CAN1, DJIMotor::CURRENT })),
    2138, 160, -160, 0,
    std::make_unique <PID> (20, 2, 0.0, 16384, 1000),
    std::make_unique <PID> (2, 0, 0, 150, 0)
);

float joint2_forward(float deg) {
    float temp_tor = 0;
    temp_tor = std::sin(deg)*SERIES_MASS*G*SERIES_JOINT2_L;
    return temp_tor;
}

Joint joint2(
    std::make_unique<MotorController>(std::make_unique <DJIMotor> (
        "joint2",
        DJIMotor::GM6020,
        (DJIMotor::Param) { 0x02, E_CAN1, DJIMotor::CURRENT })),
    3293, 180, -180, 1,
    std::make_unique <PID> (20, 2, 0.0, 16384, 1000),
    std::make_unique <PID> (2, 0, 0, 150, 0),
    joint2_forward
);

Joint joint3(
    std::make_unique<MotorController>(std::make_unique <DJIMotor> (
        "joint3",
        DJIMotor::GM6020,
        (DJIMotor::Param) { 0x03, E_CAN1, DJIMotor::CURRENT })),
    2000, 160, -160, 0,
    std::make_unique <PID> (20, 2, 0.0, 16384, 1000),
    std::make_unique <PID> (2, 0, 0, 150, 0)
);

delta::Kinematics my_delta(100,40,100,224);
delta::Dynamic my_dynamic(100,40,100,224);

float num = 114;
void get_num(bsp_uart_e e, uint8_t *s, uint16_t l) {
    float target;
    sscanf((char *) s, "%f", &target);
    num = target;
}

// 静态任务，在 CubeMX 中配置
void app_custom_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(3000);
    float pos[3];
    float rpy[3];
    float tor[3] = {0, 0, 0};;
    float deg[3];
    float target_force[3] = {0, 0, 24};
    float deg_t[3];
    float pos_t[3] = {10, -22, -200};
    DM_Motor1.enable(),DM_Motor2.enable(),DM_Motor3.enable();

    bsp_uart_set_callback(E_UART_REFEREE, get_num);

    float count = 0;
    uint8_t c_count = 0;

    while(true) {
        if(++count == 3000) {
            count = -3000;
        }
        pos_t[1] = -22.0f + std::fabs(count)/50.0f;
        pos_t[2] = -200.0f + std::fabs(count)/50.0f;

        // DM_Motor1.control(0,0,0,0,tor[0]);
        // DM_Motor2.control(0,0,0,0,tor[1]);
        // DM_Motor3.control(0,0,0,0,tor[2]);
        DM_Motor1.control(-deg_t[0],0,5,1,tor[0]);
        DM_Motor2.control(-deg_t[1],0,5,1,tor[1]);
        DM_Motor3.control(-deg_t[2],0,5,1,tor[2]);

        // joint1.joint_update(false, 0, 0);
        // joint2.joint_update(false, 0, 0);
        // joint3.joint_update(false, 0, 0);
        joint1.joint_update(true, -90, 0);
        joint2.joint_update(true, -30+std::fabs(count)/50.0f, 0);
        joint3.joint_update(true, -25+std::fabs(count)/50.0f, 0);

        deg[0] = -DM_Motor1.status.pos;
        deg[1] = -DM_Motor2.status.pos;
        deg[2] = -DM_Motor3.status.pos;
        rpy[0] = joint1.angle_;
        rpy[1] = joint2.angle_;
        rpy[2] = joint3.angle_;

        Matrixf<3, 1> deg_(deg);
        Matrixf<3, 1> t_force(target_force);
        Matrixf<3, 1> t_pos(pos_t);
        Matrixf<3, 1> pos_ = my_delta.delta_forward_clc(deg_);
        Matrixf<3, 1> tor_ = my_dynamic.tor_clc(deg_, pos_, t_force);
        Matrixf<3, 1> t_deg = my_delta.delta_inverse_clc(t_pos);

        for(uint8_t i = 0; i < 3; i++) {
            pos[i] = pos_[i][0];
            tor[i] = tor_[i][0];
            deg_t[i] = t_deg[i][0];
        }
        app_msg_vofa_send(E_UART_DEBUG,
            pos[0],
            pos[1],
            pos[2],
            pos_t[0],
            pos_t[1],
            pos_t[2]
            // DM_Motor1.status.torque,
            // DM_Motor2.status.torque,
            // DM_Motor3.status.torque,
            // rpy[0],
            // joint2.motor_ctrl_->output,
            // joint2.t_tor * GM6020_TOR_TO_COUNT,
            // joint2.motor_ctrl_->device()->angle,
            // // joint3.motor_ctrl_->angle
            // joint2.lst_online_time
            );
        OS::Task::SleepMilliseconds(1);
        if(++c_count == 10) {
            c_count = 0;
            app_msg_vofa_send(E_UART_REFEREE,
                        pos[0],
                        pos[1],
                        pos[2]
                        );
        }
        // bsp_uart_printf(E_UART_REFEREE, "hello world,%f\r\n", num);
    }
}

void app_custom_init() {
    DM_Motor1.init(),DM_Motor2.init(),DM_Motor3.init();

    DM_Motor1.enable(),DM_Motor2.enable(),DM_Motor3.enable();

    joint1.joint_init(), joint2.joint_init(), joint3.joint_init();
}

#endif
