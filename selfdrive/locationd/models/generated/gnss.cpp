#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9027589456268185175) {
   out_9027589456268185175[0] = delta_x[0] + nom_x[0];
   out_9027589456268185175[1] = delta_x[1] + nom_x[1];
   out_9027589456268185175[2] = delta_x[2] + nom_x[2];
   out_9027589456268185175[3] = delta_x[3] + nom_x[3];
   out_9027589456268185175[4] = delta_x[4] + nom_x[4];
   out_9027589456268185175[5] = delta_x[5] + nom_x[5];
   out_9027589456268185175[6] = delta_x[6] + nom_x[6];
   out_9027589456268185175[7] = delta_x[7] + nom_x[7];
   out_9027589456268185175[8] = delta_x[8] + nom_x[8];
   out_9027589456268185175[9] = delta_x[9] + nom_x[9];
   out_9027589456268185175[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7205169659528970027) {
   out_7205169659528970027[0] = -nom_x[0] + true_x[0];
   out_7205169659528970027[1] = -nom_x[1] + true_x[1];
   out_7205169659528970027[2] = -nom_x[2] + true_x[2];
   out_7205169659528970027[3] = -nom_x[3] + true_x[3];
   out_7205169659528970027[4] = -nom_x[4] + true_x[4];
   out_7205169659528970027[5] = -nom_x[5] + true_x[5];
   out_7205169659528970027[6] = -nom_x[6] + true_x[6];
   out_7205169659528970027[7] = -nom_x[7] + true_x[7];
   out_7205169659528970027[8] = -nom_x[8] + true_x[8];
   out_7205169659528970027[9] = -nom_x[9] + true_x[9];
   out_7205169659528970027[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6546852739111206180) {
   out_6546852739111206180[0] = 1.0;
   out_6546852739111206180[1] = 0;
   out_6546852739111206180[2] = 0;
   out_6546852739111206180[3] = 0;
   out_6546852739111206180[4] = 0;
   out_6546852739111206180[5] = 0;
   out_6546852739111206180[6] = 0;
   out_6546852739111206180[7] = 0;
   out_6546852739111206180[8] = 0;
   out_6546852739111206180[9] = 0;
   out_6546852739111206180[10] = 0;
   out_6546852739111206180[11] = 0;
   out_6546852739111206180[12] = 1.0;
   out_6546852739111206180[13] = 0;
   out_6546852739111206180[14] = 0;
   out_6546852739111206180[15] = 0;
   out_6546852739111206180[16] = 0;
   out_6546852739111206180[17] = 0;
   out_6546852739111206180[18] = 0;
   out_6546852739111206180[19] = 0;
   out_6546852739111206180[20] = 0;
   out_6546852739111206180[21] = 0;
   out_6546852739111206180[22] = 0;
   out_6546852739111206180[23] = 0;
   out_6546852739111206180[24] = 1.0;
   out_6546852739111206180[25] = 0;
   out_6546852739111206180[26] = 0;
   out_6546852739111206180[27] = 0;
   out_6546852739111206180[28] = 0;
   out_6546852739111206180[29] = 0;
   out_6546852739111206180[30] = 0;
   out_6546852739111206180[31] = 0;
   out_6546852739111206180[32] = 0;
   out_6546852739111206180[33] = 0;
   out_6546852739111206180[34] = 0;
   out_6546852739111206180[35] = 0;
   out_6546852739111206180[36] = 1.0;
   out_6546852739111206180[37] = 0;
   out_6546852739111206180[38] = 0;
   out_6546852739111206180[39] = 0;
   out_6546852739111206180[40] = 0;
   out_6546852739111206180[41] = 0;
   out_6546852739111206180[42] = 0;
   out_6546852739111206180[43] = 0;
   out_6546852739111206180[44] = 0;
   out_6546852739111206180[45] = 0;
   out_6546852739111206180[46] = 0;
   out_6546852739111206180[47] = 0;
   out_6546852739111206180[48] = 1.0;
   out_6546852739111206180[49] = 0;
   out_6546852739111206180[50] = 0;
   out_6546852739111206180[51] = 0;
   out_6546852739111206180[52] = 0;
   out_6546852739111206180[53] = 0;
   out_6546852739111206180[54] = 0;
   out_6546852739111206180[55] = 0;
   out_6546852739111206180[56] = 0;
   out_6546852739111206180[57] = 0;
   out_6546852739111206180[58] = 0;
   out_6546852739111206180[59] = 0;
   out_6546852739111206180[60] = 1.0;
   out_6546852739111206180[61] = 0;
   out_6546852739111206180[62] = 0;
   out_6546852739111206180[63] = 0;
   out_6546852739111206180[64] = 0;
   out_6546852739111206180[65] = 0;
   out_6546852739111206180[66] = 0;
   out_6546852739111206180[67] = 0;
   out_6546852739111206180[68] = 0;
   out_6546852739111206180[69] = 0;
   out_6546852739111206180[70] = 0;
   out_6546852739111206180[71] = 0;
   out_6546852739111206180[72] = 1.0;
   out_6546852739111206180[73] = 0;
   out_6546852739111206180[74] = 0;
   out_6546852739111206180[75] = 0;
   out_6546852739111206180[76] = 0;
   out_6546852739111206180[77] = 0;
   out_6546852739111206180[78] = 0;
   out_6546852739111206180[79] = 0;
   out_6546852739111206180[80] = 0;
   out_6546852739111206180[81] = 0;
   out_6546852739111206180[82] = 0;
   out_6546852739111206180[83] = 0;
   out_6546852739111206180[84] = 1.0;
   out_6546852739111206180[85] = 0;
   out_6546852739111206180[86] = 0;
   out_6546852739111206180[87] = 0;
   out_6546852739111206180[88] = 0;
   out_6546852739111206180[89] = 0;
   out_6546852739111206180[90] = 0;
   out_6546852739111206180[91] = 0;
   out_6546852739111206180[92] = 0;
   out_6546852739111206180[93] = 0;
   out_6546852739111206180[94] = 0;
   out_6546852739111206180[95] = 0;
   out_6546852739111206180[96] = 1.0;
   out_6546852739111206180[97] = 0;
   out_6546852739111206180[98] = 0;
   out_6546852739111206180[99] = 0;
   out_6546852739111206180[100] = 0;
   out_6546852739111206180[101] = 0;
   out_6546852739111206180[102] = 0;
   out_6546852739111206180[103] = 0;
   out_6546852739111206180[104] = 0;
   out_6546852739111206180[105] = 0;
   out_6546852739111206180[106] = 0;
   out_6546852739111206180[107] = 0;
   out_6546852739111206180[108] = 1.0;
   out_6546852739111206180[109] = 0;
   out_6546852739111206180[110] = 0;
   out_6546852739111206180[111] = 0;
   out_6546852739111206180[112] = 0;
   out_6546852739111206180[113] = 0;
   out_6546852739111206180[114] = 0;
   out_6546852739111206180[115] = 0;
   out_6546852739111206180[116] = 0;
   out_6546852739111206180[117] = 0;
   out_6546852739111206180[118] = 0;
   out_6546852739111206180[119] = 0;
   out_6546852739111206180[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1612449122734519935) {
   out_1612449122734519935[0] = dt*state[3] + state[0];
   out_1612449122734519935[1] = dt*state[4] + state[1];
   out_1612449122734519935[2] = dt*state[5] + state[2];
   out_1612449122734519935[3] = state[3];
   out_1612449122734519935[4] = state[4];
   out_1612449122734519935[5] = state[5];
   out_1612449122734519935[6] = dt*state[7] + state[6];
   out_1612449122734519935[7] = dt*state[8] + state[7];
   out_1612449122734519935[8] = state[8];
   out_1612449122734519935[9] = state[9];
   out_1612449122734519935[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8155049146706996953) {
   out_8155049146706996953[0] = 1;
   out_8155049146706996953[1] = 0;
   out_8155049146706996953[2] = 0;
   out_8155049146706996953[3] = dt;
   out_8155049146706996953[4] = 0;
   out_8155049146706996953[5] = 0;
   out_8155049146706996953[6] = 0;
   out_8155049146706996953[7] = 0;
   out_8155049146706996953[8] = 0;
   out_8155049146706996953[9] = 0;
   out_8155049146706996953[10] = 0;
   out_8155049146706996953[11] = 0;
   out_8155049146706996953[12] = 1;
   out_8155049146706996953[13] = 0;
   out_8155049146706996953[14] = 0;
   out_8155049146706996953[15] = dt;
   out_8155049146706996953[16] = 0;
   out_8155049146706996953[17] = 0;
   out_8155049146706996953[18] = 0;
   out_8155049146706996953[19] = 0;
   out_8155049146706996953[20] = 0;
   out_8155049146706996953[21] = 0;
   out_8155049146706996953[22] = 0;
   out_8155049146706996953[23] = 0;
   out_8155049146706996953[24] = 1;
   out_8155049146706996953[25] = 0;
   out_8155049146706996953[26] = 0;
   out_8155049146706996953[27] = dt;
   out_8155049146706996953[28] = 0;
   out_8155049146706996953[29] = 0;
   out_8155049146706996953[30] = 0;
   out_8155049146706996953[31] = 0;
   out_8155049146706996953[32] = 0;
   out_8155049146706996953[33] = 0;
   out_8155049146706996953[34] = 0;
   out_8155049146706996953[35] = 0;
   out_8155049146706996953[36] = 1;
   out_8155049146706996953[37] = 0;
   out_8155049146706996953[38] = 0;
   out_8155049146706996953[39] = 0;
   out_8155049146706996953[40] = 0;
   out_8155049146706996953[41] = 0;
   out_8155049146706996953[42] = 0;
   out_8155049146706996953[43] = 0;
   out_8155049146706996953[44] = 0;
   out_8155049146706996953[45] = 0;
   out_8155049146706996953[46] = 0;
   out_8155049146706996953[47] = 0;
   out_8155049146706996953[48] = 1;
   out_8155049146706996953[49] = 0;
   out_8155049146706996953[50] = 0;
   out_8155049146706996953[51] = 0;
   out_8155049146706996953[52] = 0;
   out_8155049146706996953[53] = 0;
   out_8155049146706996953[54] = 0;
   out_8155049146706996953[55] = 0;
   out_8155049146706996953[56] = 0;
   out_8155049146706996953[57] = 0;
   out_8155049146706996953[58] = 0;
   out_8155049146706996953[59] = 0;
   out_8155049146706996953[60] = 1;
   out_8155049146706996953[61] = 0;
   out_8155049146706996953[62] = 0;
   out_8155049146706996953[63] = 0;
   out_8155049146706996953[64] = 0;
   out_8155049146706996953[65] = 0;
   out_8155049146706996953[66] = 0;
   out_8155049146706996953[67] = 0;
   out_8155049146706996953[68] = 0;
   out_8155049146706996953[69] = 0;
   out_8155049146706996953[70] = 0;
   out_8155049146706996953[71] = 0;
   out_8155049146706996953[72] = 1;
   out_8155049146706996953[73] = dt;
   out_8155049146706996953[74] = 0;
   out_8155049146706996953[75] = 0;
   out_8155049146706996953[76] = 0;
   out_8155049146706996953[77] = 0;
   out_8155049146706996953[78] = 0;
   out_8155049146706996953[79] = 0;
   out_8155049146706996953[80] = 0;
   out_8155049146706996953[81] = 0;
   out_8155049146706996953[82] = 0;
   out_8155049146706996953[83] = 0;
   out_8155049146706996953[84] = 1;
   out_8155049146706996953[85] = dt;
   out_8155049146706996953[86] = 0;
   out_8155049146706996953[87] = 0;
   out_8155049146706996953[88] = 0;
   out_8155049146706996953[89] = 0;
   out_8155049146706996953[90] = 0;
   out_8155049146706996953[91] = 0;
   out_8155049146706996953[92] = 0;
   out_8155049146706996953[93] = 0;
   out_8155049146706996953[94] = 0;
   out_8155049146706996953[95] = 0;
   out_8155049146706996953[96] = 1;
   out_8155049146706996953[97] = 0;
   out_8155049146706996953[98] = 0;
   out_8155049146706996953[99] = 0;
   out_8155049146706996953[100] = 0;
   out_8155049146706996953[101] = 0;
   out_8155049146706996953[102] = 0;
   out_8155049146706996953[103] = 0;
   out_8155049146706996953[104] = 0;
   out_8155049146706996953[105] = 0;
   out_8155049146706996953[106] = 0;
   out_8155049146706996953[107] = 0;
   out_8155049146706996953[108] = 1;
   out_8155049146706996953[109] = 0;
   out_8155049146706996953[110] = 0;
   out_8155049146706996953[111] = 0;
   out_8155049146706996953[112] = 0;
   out_8155049146706996953[113] = 0;
   out_8155049146706996953[114] = 0;
   out_8155049146706996953[115] = 0;
   out_8155049146706996953[116] = 0;
   out_8155049146706996953[117] = 0;
   out_8155049146706996953[118] = 0;
   out_8155049146706996953[119] = 0;
   out_8155049146706996953[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3373808474481276122) {
   out_3373808474481276122[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8860199885964646433) {
   out_8860199885964646433[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8860199885964646433[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8860199885964646433[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8860199885964646433[3] = 0;
   out_8860199885964646433[4] = 0;
   out_8860199885964646433[5] = 0;
   out_8860199885964646433[6] = 1;
   out_8860199885964646433[7] = 0;
   out_8860199885964646433[8] = 0;
   out_8860199885964646433[9] = 0;
   out_8860199885964646433[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5196876421747971199) {
   out_5196876421747971199[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_9149420611786637566) {
   out_9149420611786637566[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9149420611786637566[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9149420611786637566[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9149420611786637566[3] = 0;
   out_9149420611786637566[4] = 0;
   out_9149420611786637566[5] = 0;
   out_9149420611786637566[6] = 1;
   out_9149420611786637566[7] = 0;
   out_9149420611786637566[8] = 0;
   out_9149420611786637566[9] = 1;
   out_9149420611786637566[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7061299658242464651) {
   out_7061299658242464651[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_331468830178364318) {
   out_331468830178364318[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[6] = 0;
   out_331468830178364318[7] = 1;
   out_331468830178364318[8] = 0;
   out_331468830178364318[9] = 0;
   out_331468830178364318[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7061299658242464651) {
   out_7061299658242464651[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_331468830178364318) {
   out_331468830178364318[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_331468830178364318[6] = 0;
   out_331468830178364318[7] = 1;
   out_331468830178364318[8] = 0;
   out_331468830178364318[9] = 0;
   out_331468830178364318[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9027589456268185175) {
  err_fun(nom_x, delta_x, out_9027589456268185175);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7205169659528970027) {
  inv_err_fun(nom_x, true_x, out_7205169659528970027);
}
void gnss_H_mod_fun(double *state, double *out_6546852739111206180) {
  H_mod_fun(state, out_6546852739111206180);
}
void gnss_f_fun(double *state, double dt, double *out_1612449122734519935) {
  f_fun(state,  dt, out_1612449122734519935);
}
void gnss_F_fun(double *state, double dt, double *out_8155049146706996953) {
  F_fun(state,  dt, out_8155049146706996953);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3373808474481276122) {
  h_6(state, sat_pos, out_3373808474481276122);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8860199885964646433) {
  H_6(state, sat_pos, out_8860199885964646433);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5196876421747971199) {
  h_20(state, sat_pos, out_5196876421747971199);
}
void gnss_H_20(double *state, double *sat_pos, double *out_9149420611786637566) {
  H_20(state, sat_pos, out_9149420611786637566);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7061299658242464651) {
  h_7(state, sat_pos_vel, out_7061299658242464651);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_331468830178364318) {
  H_7(state, sat_pos_vel, out_331468830178364318);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7061299658242464651) {
  h_21(state, sat_pos_vel, out_7061299658242464651);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_331468830178364318) {
  H_21(state, sat_pos_vel, out_331468830178364318);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
