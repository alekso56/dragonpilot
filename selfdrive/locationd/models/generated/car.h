#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5829269461866699088);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2615413374656959367);
void car_H_mod_fun(double *state, double *out_6763671061317272364);
void car_f_fun(double *state, double dt, double *out_990004811134166701);
void car_F_fun(double *state, double dt, double *out_6322414328777713969);
void car_h_25(double *state, double *unused, double *out_3346788960512997050);
void car_H_25(double *state, double *unused, double *out_4465233356863455756);
void car_h_24(double *state, double *unused, double *out_92270251985139515);
void car_H_24(double *state, double *unused, double *out_907307758061132620);
void car_h_30(double *state, double *unused, double *out_1967819088175988157);
void car_H_30(double *state, double *unused, double *out_2451456984628160999);
void car_h_26(double *state, double *unused, double *out_3327446619185357985);
void car_H_26(double *state, double *unused, double *out_8206736675737511980);
void car_h_27(double *state, double *unused, double *out_4290298813414659485);
void car_H_27(double *state, double *unused, double *out_276693672827736088);
void car_h_29(double *state, double *unused, double *out_6969492355836833802);
void car_H_29(double *state, double *unused, double *out_1436669054041814945);
void car_h_28(double *state, double *unused, double *out_1352431057666297350);
void car_H_28(double *state, double *unused, double *out_6519068071111345519);
void car_h_31(double *state, double *unused, double *out_1944824502362540082);
void car_H_31(double *state, double *unused, double *out_4434587394986495328);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}