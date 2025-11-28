//
// Created by guan on 2025/11/5.
// Refactored from app_custom.h
//

#pragma once

#include <matrix.h>

namespace Algorithm {
namespace delta {
//统一使用弧度制
class Kinematics {
public:
    Kinematics();
    Kinematics(float const R_, float const r_, float const L_, float const l_)
    : R(R_)
    , r(r_)
    , L(L_)
    , l(l_) {
        float data1[3] = { R_, 0, 0 }, data2[3] = { -r_, 0, 0 };
        Matrixf<3, 1> temp1(data1), temp2(data2);
        for(uint8_t i = 0; i < 3; i++) {
            pos_oc[i]  = temp1.rot_z(phi[i]) * temp1;
            pos_ap[i]  = temp2.rot_z(phi[i]) * temp2;
            theta_i[i] = 0;
        }
    };

    Matrixf<3, 1> delta_forward_clc(const Matrixf<3, 1> &_theta) {
        theta_i[0] = _theta[0][0], theta_i[1] = _theta[1][0], theta_i[2] = _theta[2][0];
        for(uint8_t i = 0; i < 3; i++) {
            get_cbi(theta_i[i], phi[i], &pos_cb[i]);
            pos_od[i] = pos_oc[i] + pos_cb[i] + pos_ap[i];
        }

        vector_d12 = pos_od[1] - pos_od[0];
        vector_d23 = pos_od[2] - pos_od[1];
        vector_d13 = pos_od[2] - pos_od[0];

        vector_t.vector_cross_multiple(vector_d13, vector_d12);
        vector_t /= vector_t.norm();

        vector_n.vector_cross_multiple(vector_d23, vector_t);
        vector_n /= vector_n.norm();

        float a                 = vector_d13.norm();
        float b                 = vector_d12.norm();
        float c                 = vector_d23.norm();
        float p                 = (a + b + c) / 2;
        float s                 = sqrt(p * (p - a) * (p - c) * (p - b));
        float R_D               = a * b * c / (4 * s);
        Matrixf<3, 1> vector_ef = sqrt(R_D * R_D - c * c / 4) * vector_n;
        Matrixf<3, 1> vector_fp = sqrt(l * l - R_D * R_D) * vector_t;
        pos_op                  = (pos_od[1] + pos_od[2]) / 2 + vector_ef + vector_fp;

        return pos_op;
    }

    Matrixf<3, 1> delta_inverse_clc(const Matrixf<3, 1> &_pos) {
        float pos[3] = { _pos[0][0], _pos[1][0], _pos[2][0] };
        float A[3] = {}, B[3] = {}, C[3] = {}, t[3] = {}, theta[3] = {};
        float temp = (R - r) * (R - r) + pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2] + L * L - l * l;
        for(uint8_t i = 0; i < 3; i++) {
            A[i] = temp - 2 * pos[0] * cos(phi[i]) * (R - r) - 2 * pos[1] * sin(phi[i]) * (R - r) - 2 * (R - r) * L +
                   2 * pos[0] * cos(phi[i]) * L + 2 * pos[1] * sin(phi[i]) * L;
            B[i]     = 4 * L * pos[2];
            C[i]     = A[i] - 2 * (2 * pos[0] * cos(phi[i]) * L + 2 * pos[1] * sin(phi[i]) * L - 2 * (R - r) * L);
            t[i]     = (-B[i] - sqrt(B[i] * B[i] - 4 * A[i] * C[i])) / (2 * A[i]);
            theta[i] = 2.0f * atan2(t[i], 1.0f);
        }
        return { theta };
    }

private:
    void get_cbi(float _theta, float _phi, Matrixf<3, 1> *mat) {
        float data[3] = { L * cos(_theta), 0, -L * sin(_theta) };
        Matrixf<3, 1> temp(data);
        *mat = temp.rot_z(_phi) * temp;
    }
    float R, r, L, l;
    const float phi[3] = {
        0,
        2.0f * M_PI / 3.0f,
        4.0f * M_PI / 3.0f,
    };
    float theta_i[3];
    Matrixf<3, 1> pos_oc[3];
    Matrixf<3, 1> pos_cb[3];
    Matrixf<3, 1> pos_ap[3];
    Matrixf<3, 1> pos_od[3];
    Matrixf<3, 1> vector_d13, vector_d12, vector_d23;
    Matrixf<3, 1> vector_t, vector_n;
    Matrixf<3, 1> pos_op;
};

class Dynamic {
public:
    Dynamic();
    Dynamic(float const R_, float const r_, float const L_, float const l_)
    : R(R_)
    , r(r_)
    , L(L_)
    , l(l_) {
        float data1[3] = { R_, 0, 0 }, data2[3] = { r_, 0, 0 };
        Matrixf<3, 1> temp1(data1), temp2(data2);
        for(uint8_t i = 0; i < 3; i++) {
            pos_oc[i]  = temp1.rot_z(phi[i]) * temp1;
            pos_pa[i]  = temp2.rot_z(phi[i]) * temp2;
            theta_i[i] = 0;
        }
    }

    Matrixf<3, 1> tor_clc(const Matrixf<3, 1> &_theta, const Matrixf<3, 1> &vector_OP, const Matrixf<3, 1> &force) {
        Matrixf<3, 1> vec_op = vector_OP, vec_force = force;
        theta_i[0] = _theta[0][0], theta_i[1] = _theta[1][0], theta_i[2] = _theta[2][0];
        Matrixf<1, 3> t_s[3];
        for(uint8_t i = 0; i < 3; i++) {
            get_bi(theta_i[i], phi[i], &pos_b[i]);
            get_cbi(theta_i[i], phi[i], &pos_cb[i]);
            pos_s[i] = (vec_op + pos_pa[i] - pos_oc[i] - pos_cb[i]) / 1000;
            t_s[i]   = pos_s[i].trans();
        }

        Matrixf<3, 3> temp1 = matrixf::vertcat(matrixf::vertcat(t_s[0], t_s[1]), t_s[2]);
        temp1               = matrixf::inv(temp1);
        Matrixf<3, 3> temp2 = matrixf::zeros<3, 3>();
        for(uint8_t i = 0; i < 3; i++) {
            Matrixf<1, 1> temp = t_s[i] * pos_b[i] / 1000;
            temp2[i][i]        = temp[0][0];
        }
        Matrixf<3, 3> jacobi = (temp1) * (temp2);
        Matrixf<1, 3> f      = vec_force.trans();
        tor                  = f * jacobi;

        return tor.trans();
    }

private:
    void get_bi(float _theta, float _phi, Matrixf<3, 1> *mat) const {
        //这里正负号存疑
        float data[3] = { L * sin(_theta), 0, L * cos(_theta) };
        Matrixf<3, 1> temp(data);
        *mat = temp.rot_z(_phi) * temp;
    }
    void get_cbi(float _theta, float _phi, Matrixf<3, 1> *mat) const {
        float data[3] = { L * cos(_theta), 0, -L * sin(_theta) };
        Matrixf<3, 1> temp(data);
        *mat = temp.rot_z(_phi) * temp;
    }
    float R, r, L, l;
    const float phi[3] = {
        0,
        2.0f * M_PI / 3.0f,
        4.0f * M_PI / 3.0f,
    };
    float theta_i[3];
    Matrixf<3, 1> pos_oc[3];
    Matrixf<3, 1> pos_b[3]; //x+bθ其中的b
    Matrixf<3, 1> pos_cb[3];
    Matrixf<3, 1> pos_pa[3];
    Matrixf<3, 1> pos_s[3];
    Matrixf<1, 3> tor;
};
} // namespace delta
} // namespace Algorithm
