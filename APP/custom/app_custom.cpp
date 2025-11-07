//
// Created by guan on 2025/11/5.
//

#include "app_custom.h"
#include "app_sys.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "dev_motor_dm.h"
// #include "app_custom_series.h"
#include "app_msg.h"
#include "ctrl_motor_base_pid.h"

#include "app_sys.h"
#include "sys_task.h"

#ifdef COMPILE_CUSTOM

Motor::DMMotor DM_Motor1("Motor1",Motor::DMMotor::J4310,{
    .slave_id = 0x01,
    .master_id = 0x11,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
Motor::DMMotor DM_Motor2("Motor2",Motor::DMMotor::J4310,{
    .slave_id = 0x02,
    .master_id = 0x12,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
Motor::DMMotor DM_Motor3("Motor3",Motor::DMMotor::J4310,{
    .slave_id = 0x03,
    .master_id = 0x13,
    .port = E_CAN2,
    .mode = Motor::DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});

delta::Kinematics my_delta(100,40,100,224);
delta::Dynamic my_dynamic(100,40,100,224);
// Controller::PID pid_x, pid_y, pid_z;

// 静态任务，在 CubeMX 中配置
void app_custom_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(3000);
    float pos[3];
    // float rpy[3];
    float tor[3] = {0, 0, 0};;
    float deg[3];
    float target_force[3] = {0, 0, 24};
    float deg_t[3];
    float pos_t[3] = {29, -22, -220};
    // pid_x.set_para(0.005, 0, 1, 1, 0);
    // pid_y.set_para(0.005, 0, 1, 1, 0);
    // pid_z.set_para(0.005, 0, 1, 1, 0);

    while(true) {

        DM_Motor1.control(deg_t[0],1,1,1,tor[0]);
        DM_Motor2.control(deg_t[1],1,1,1,tor[1]);
        DM_Motor3.control(deg_t[2],1,1,1,tor[2]);
        deg[0] = -DM_Motor1.status.pos;
        deg[1] = -DM_Motor2.status.pos;
        deg[2] = -DM_Motor3.status.pos;

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
            // rpy[0],
            // rpy[1],
            // rpy[2],
            pos[0],
            pos[1],
            pos[2],
            1.0*tor[0],
            1.0*tor[1],
            1.0*tor[2]
            // pid_x.update(deg[0], deg_t[0]),
            // pid_y.update(deg[1], deg_t[1]),
            // pid_z.update(deg[2], deg_t[2])
            // DM_Motor1.status.pos,
            // DM_Motor2.status.pos,
            // DM_Motor3.status.pos
            );
        OS::Task::SleepMilliseconds(1);
    }
}

void app_custom_init() {
    DM_Motor1.init(),DM_Motor2.init(),DM_Motor3.init();

    DM_Motor1.enable(),DM_Motor2.enable(),DM_Motor3.enable();
}

#endif
