#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9027589456268185175);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7205169659528970027);
void gnss_H_mod_fun(double *state, double *out_6546852739111206180);
void gnss_f_fun(double *state, double dt, double *out_1612449122734519935);
void gnss_F_fun(double *state, double dt, double *out_8155049146706996953);
void gnss_h_6(double *state, double *sat_pos, double *out_3373808474481276122);
void gnss_H_6(double *state, double *sat_pos, double *out_8860199885964646433);
void gnss_h_20(double *state, double *sat_pos, double *out_5196876421747971199);
void gnss_H_20(double *state, double *sat_pos, double *out_9149420611786637566);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7061299658242464651);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_331468830178364318);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7061299658242464651);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_331468830178364318);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}