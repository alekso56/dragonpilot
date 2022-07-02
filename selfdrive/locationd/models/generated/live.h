#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1356256294732298968);
void live_err_fun(double *nom_x, double *delta_x, double *out_3067371793495545137);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5834301181090248469);
void live_H_mod_fun(double *state, double *out_5519691553496756147);
void live_f_fun(double *state, double dt, double *out_4962418025735912644);
void live_F_fun(double *state, double dt, double *out_7092119509530607508);
void live_h_4(double *state, double *unused, double *out_2304202517429654977);
void live_H_4(double *state, double *unused, double *out_4543373436150880457);
void live_h_9(double *state, double *unused, double *out_6924777554652844558);
void live_H_9(double *state, double *unused, double *out_7432234988430959799);
void live_h_10(double *state, double *unused, double *out_7119768345505507454);
void live_H_10(double *state, double *unused, double *out_7764892465746032956);
void live_h_12(double *state, double *unused, double *out_2658616884484351056);
void live_H_12(double *state, double *unused, double *out_6236242323876220667);
void live_h_35(double *state, double *unused, double *out_1101415130013129771);
void live_H_35(double *state, double *unused, double *out_7910035493523487833);
void live_h_32(double *state, double *unused, double *out_2302632350581349354);
void live_H_32(double *state, double *unused, double *out_2310625080656405600);
void live_h_13(double *state, double *unused, double *out_1611300704087694343);
void live_H_13(double *state, double *unused, double *out_4680254586320445968);
void live_h_14(double *state, double *unused, double *out_6924777554652844558);
void live_H_14(double *state, double *unused, double *out_7432234988430959799);
void live_h_33(double *state, double *unused, double *out_208123134487387152);
void live_H_33(double *state, double *unused, double *out_7386151575547206179);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}