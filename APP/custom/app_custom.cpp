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

// 静态任务，在 CubeMX 中配置
void app_custom_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    OS::Task::SleepMilliseconds(3000);
    float pos[3];
    float rpy[3];
    float tor[3];
    float deg[3];
    float deg_test[3];
    float target_force[3] = {0, 0, 24};

    while(true) {

        DM_Motor1.control(0,0,0,0,tor[0]);
        DM_Motor2.control(0,0,0,0,tor[1]);
        DM_Motor3.control(0,0,0,0,tor[2]);
        deg[0] = -DM_Motor1.status.pos;
        deg[1] = -DM_Motor2.status.pos;
        deg[2] = -DM_Motor3.status.pos;
        my_delta.delta_forward_clc(deg[0],deg[1],deg[2],pos);
        my_dynamic.tor_clc(deg[0],deg[1],deg[2],pos,target_force[0],target_force[1],target_force[2],tor);

        my_delta.delta_inverse_clc(pos, deg_test);

        app_msg_vofa_send(E_UART_DEBUG,
            // rpy[0],
            // rpy[1],
            // rpy[2],
            // pos[0],
            // pos[1],
            // pos[2],
            1.0*tor[0],
            1.0*tor[1],
            1.0*tor[2],
            DM_Motor2.status.torque,
            DM_Motor1.status.pos,
            DM_Motor2.status.pos,
            DM_Motor3.status.pos
            );
        OS::Task::SleepMilliseconds(1);
    }
}

void app_custom_init() {
    DM_Motor1.init(),DM_Motor2.init(),DM_Motor3.init();

    DM_Motor1.enable(),DM_Motor2.enable(),DM_Motor3.enable();
}

#endif
