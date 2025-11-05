//
// Created by guan on 2025/11/5.
//

#pragma once

#include "app_conf.h"
#include <matrix.h>

#ifdef __cplusplus
extern "C" {
#endif

    void app_custom_init();
    void app_custom_task(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/*
 *
 */
namespace delta {
    //统一使用弧度制
    class Kinematics {
        public:
        Kinematics();
        Kinematics(float const R_, float const r_, float const L_, float const l_) : R(R_), r(r_), L(L_), l(l_) {
            theta_1 = theta_2 = theta_3 = 0;
            float data[3] = {R_,0,0};
            Matrixf<3,1> temp(data);
            pos_oc1 =  temp.rot_z(phi[0]) * temp;
            pos_oc2 =  temp.rot_z(phi[1]) * temp;
            pos_oc3 =  temp.rot_z(phi[2]) * temp;
            data[0] = -r_;
            Matrixf<3,1> temp2(data);
            pos_a1p =  temp.rot_z(phi[0]) * temp2;
            pos_a2p =  temp.rot_z(phi[1]) * temp2;
            pos_a3p =  temp.rot_z(phi[2]) * temp2;
        };

        //将来想改为Matrixf<3,1>类型
        void delta_forward_clc(float _theta1, float _theta2, float _theta3, float *position) {
            theta_1 = _theta1 ,theta_2 = _theta2 ,theta_3 = _theta3;
            get_cbi(theta_1, phi[0], &pos_cb1);
            get_cbi(theta_2, phi[1], &pos_cb2);
            get_cbi(theta_3, phi[2], &pos_cb3);

            pos_od1 = pos_oc1 + pos_cb1 + pos_a1p;
            pos_od2 = pos_oc2 + pos_cb2 + pos_a2p;
            pos_od3 = pos_oc3 + pos_cb3 + pos_a3p;

            vector_d12 = pos_od2 - pos_od1;
            vector_d23 = pos_od3 - pos_od2;
            vector_d13 = pos_od3 - pos_od1;

            vector_t.vector_cross_multiple(vector_d13,vector_d12);
            vector_t /= vector_t.norm();

            vector_n.vector_cross_multiple(vector_d23,vector_t);
            vector_n /= vector_n.norm();

            float a = vector_d13.norm();
            float b = vector_d12.norm();
            float c = vector_d23.norm();
            float p = (a+b+c)/2;
            float s = sqrt(p*(p-a)*(p-c)*(p-b));
            float R_D = a*b*c/(4*s);
            Matrixf<3,1> vector_ef = sqrt(R_D*R_D - c*c/4)*vector_n;
            Matrixf<3,1> vector_fp = sqrt(l*l - R_D*R_D)*vector_t;
            pos_op = (pos_od2+pos_od3)/2+vector_ef+vector_fp;
            position[0] = pos_op[0][0];
            position[1] = pos_op[1][0];
            position[2] = pos_op[2][0];
        }

        void delta_inverse_clc(float *pos, float *theta) {
            float A[3] = {}, B[3] = {}, C[3] = {}, t[3] = {};
            float temp = (R-r)*(R-r) + pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2] + L*L - l*l;
            for(uint8_t i = 0; i<3; i++) {
                A[i] = temp - 2*pos[0]*cos(phi[i])*(R-r) - 2*pos[1]*sin(phi[i])*(R-r) - 2*(R-r)*L + 2*pos[0]*cos(phi[i])*L + 2*pos[1]*sin(phi[i])*L;
                B[i] = 4*L*pos[2];
                C[i] = A[i] - 2*(2*pos[0]*cos(phi[i])*L + 2*pos[1]*sin(phi[i])*L - 2*(R-r)*L);
                t[i] = (-B[i] - sqrt(B[i]*B[i] - 4*A[i]*C[i])) / (2*A[i]);
                theta[i] = 2.0f * atan2(t[i], 1.0f);
            }
        }

    private:
        void get_cbi(float _theta, float _phi, Matrixf<3,1> *mat) {
            float data[3] = {L*cos(_theta),0,-L*sin(_theta)};
            Matrixf<3,1> temp(data);
            *mat = temp.rot_z(_phi) * temp;
        }
        float R,r,L,l;
        const float phi[3] = {0,
                        2.0f * M_PI / 3.0f,
                        4.0f * M_PI / 3.0f,};
        float theta_1,theta_2,theta_3;
        Matrixf<3,1> pos_oc1, pos_oc2, pos_oc3;
        Matrixf<3,1> pos_cb1, pos_cb2, pos_cb3;
        Matrixf<3,1> pos_a1p, pos_a2p, pos_a3p;
        Matrixf<3,1> pos_od1, pos_od2, pos_od3;
        Matrixf<3,1> vector_d13, vector_d12, vector_d23;
        Matrixf<3,1> vector_t, vector_n;
        Matrixf<3,1> pos_op;
    };
    class Dynamic {
        public:
        Dynamic();
        Dynamic(float const R_, float const r_, float const L_, float const l_): R(R_), r(r_), L(L_), l(l_) {
            phi_1 = 0;
            phi_2 = PI*2/3;
            phi_3 = PI*4/3;
            theta_1 = theta_2 = theta_3 = 0;
            float data[3] = {R_,0,0};
            Matrixf<3,1> temp(data);
            pos_oc1 =  temp.rot_z(phi_1) * temp;
            pos_oc2 =  temp.rot_z(phi_2) * temp;
            pos_oc3 =  temp.rot_z(phi_3) * temp;
            data[0] = r_;
            Matrixf<3,1> temp2(data);
            pos_pa1 =  temp.rot_z(phi_1) * temp2;
            pos_pa2 =  temp.rot_z(phi_2) * temp2;
            pos_pa3 =  temp.rot_z(phi_3) * temp2;
        }

        //打算换成Matrixf<3,1>
        void tor_clc(float _theta1, float _theta2, float _theta3, float *pos, float force_x, float force_y, float force_z, float *answer) {
            theta_1 = (_theta1), theta_2 = (_theta2), theta_3 = (_theta3);
            Matrixf<3,1> vector_OP(pos);
            get_si(phi_1,theta_1,&pos_b1,&pos_cb1,&pos_s1,pos_pa1,pos_oc1,vector_OP);
            get_si(phi_2,theta_2,&pos_b2,&pos_cb2,&pos_s2,pos_pa2,pos_oc2,vector_OP);
            get_si(phi_3,theta_3,&pos_b3,&pos_cb3,&pos_s3,pos_pa3,pos_oc3,vector_OP);
            Matrixf<1,3> t_s1 = pos_s1.trans();
            Matrixf<1,3> t_s2 = pos_s2.trans();
            Matrixf<1,3> t_s3 = pos_s3.trans();
            Matrixf<3,3> temp1 = matrixf::vertcat(matrixf::vertcat(t_s1, t_s2), t_s3);
            temp1 = matrixf::inv(temp1);
            Matrixf<1,1> temp;
            Matrixf<3,3> temp2 = matrixf::zeros<3,3>();
            temp = t_s1*pos_b1/1000;
            temp2[0][0] = temp[0][0];
            temp = t_s2*pos_b2/1000;
            temp2[1][1] = temp[0][0];
            temp = t_s3*pos_b3/1000;
            temp2[2][2] = temp[0][0];
            Matrixf<3,3> jacobi = (temp1)*(temp2);
            float force[3] = {force_x,force_y,force_z};
            Matrixf<1,3> f(force);
            tor = f*jacobi;
            answer[0] = tor[0][0];
            answer[1] = tor[0][1];
            answer[2] = tor[0][2];
        }

    private:
        void get_bi(float _theta, float _phi, Matrixf<3,1> *mat) {
            //这里正负号存疑
            float data[3] = {L*sin(_theta),0,L*cos(_theta)};
            Matrixf<3,1> temp(data);
            *mat = temp.rot_z(_phi) * temp;
        }
        void get_cbi(float _theta, float _phi, Matrixf<3,1> *mat) {
            float data[3] = {L*cos(_theta),0,-L*sin(_theta)};
            Matrixf<3,1> temp(data);
            *mat = temp.rot_z(_phi) * temp;
        }
        void get_si( float _phi, float _theta, Matrixf<3,1> *b,Matrixf<3,1> *cb, Matrixf<3,1> *si, Matrixf<3,1> pa, Matrixf<3,1> oc,Matrixf<3,1> op) {
            get_bi(_theta,_phi,b);
            get_cbi(_theta,_phi,cb);
            *si = (op + pa - oc - *cb)/1000;
        }
        float R,r,L,l;
        float phi_1,phi_2,phi_3;
        float theta_1,theta_2,theta_3;
        Matrixf<3,1> pos_oc1, pos_oc2, pos_oc3;
        Matrixf<3,1> pos_b1, pos_b2, pos_b3;
        Matrixf<3,1> pos_cb1, pos_cb2, pos_cb3;
        Matrixf<3,1> pos_pa1, pos_pa2, pos_pa3;
        Matrixf<3,1> pos_s1, pos_s2, pos_s3;
        Matrixf<1,3> tor;
    };
}
#endif

