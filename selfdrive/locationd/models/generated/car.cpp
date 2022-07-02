#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5829269461866699088) {
   out_5829269461866699088[0] = delta_x[0] + nom_x[0];
   out_5829269461866699088[1] = delta_x[1] + nom_x[1];
   out_5829269461866699088[2] = delta_x[2] + nom_x[2];
   out_5829269461866699088[3] = delta_x[3] + nom_x[3];
   out_5829269461866699088[4] = delta_x[4] + nom_x[4];
   out_5829269461866699088[5] = delta_x[5] + nom_x[5];
   out_5829269461866699088[6] = delta_x[6] + nom_x[6];
   out_5829269461866699088[7] = delta_x[7] + nom_x[7];
   out_5829269461866699088[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2615413374656959367) {
   out_2615413374656959367[0] = -nom_x[0] + true_x[0];
   out_2615413374656959367[1] = -nom_x[1] + true_x[1];
   out_2615413374656959367[2] = -nom_x[2] + true_x[2];
   out_2615413374656959367[3] = -nom_x[3] + true_x[3];
   out_2615413374656959367[4] = -nom_x[4] + true_x[4];
   out_2615413374656959367[5] = -nom_x[5] + true_x[5];
   out_2615413374656959367[6] = -nom_x[6] + true_x[6];
   out_2615413374656959367[7] = -nom_x[7] + true_x[7];
   out_2615413374656959367[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6763671061317272364) {
   out_6763671061317272364[0] = 1.0;
   out_6763671061317272364[1] = 0;
   out_6763671061317272364[2] = 0;
   out_6763671061317272364[3] = 0;
   out_6763671061317272364[4] = 0;
   out_6763671061317272364[5] = 0;
   out_6763671061317272364[6] = 0;
   out_6763671061317272364[7] = 0;
   out_6763671061317272364[8] = 0;
   out_6763671061317272364[9] = 0;
   out_6763671061317272364[10] = 1.0;
   out_6763671061317272364[11] = 0;
   out_6763671061317272364[12] = 0;
   out_6763671061317272364[13] = 0;
   out_6763671061317272364[14] = 0;
   out_6763671061317272364[15] = 0;
   out_6763671061317272364[16] = 0;
   out_6763671061317272364[17] = 0;
   out_6763671061317272364[18] = 0;
   out_6763671061317272364[19] = 0;
   out_6763671061317272364[20] = 1.0;
   out_6763671061317272364[21] = 0;
   out_6763671061317272364[22] = 0;
   out_6763671061317272364[23] = 0;
   out_6763671061317272364[24] = 0;
   out_6763671061317272364[25] = 0;
   out_6763671061317272364[26] = 0;
   out_6763671061317272364[27] = 0;
   out_6763671061317272364[28] = 0;
   out_6763671061317272364[29] = 0;
   out_6763671061317272364[30] = 1.0;
   out_6763671061317272364[31] = 0;
   out_6763671061317272364[32] = 0;
   out_6763671061317272364[33] = 0;
   out_6763671061317272364[34] = 0;
   out_6763671061317272364[35] = 0;
   out_6763671061317272364[36] = 0;
   out_6763671061317272364[37] = 0;
   out_6763671061317272364[38] = 0;
   out_6763671061317272364[39] = 0;
   out_6763671061317272364[40] = 1.0;
   out_6763671061317272364[41] = 0;
   out_6763671061317272364[42] = 0;
   out_6763671061317272364[43] = 0;
   out_6763671061317272364[44] = 0;
   out_6763671061317272364[45] = 0;
   out_6763671061317272364[46] = 0;
   out_6763671061317272364[47] = 0;
   out_6763671061317272364[48] = 0;
   out_6763671061317272364[49] = 0;
   out_6763671061317272364[50] = 1.0;
   out_6763671061317272364[51] = 0;
   out_6763671061317272364[52] = 0;
   out_6763671061317272364[53] = 0;
   out_6763671061317272364[54] = 0;
   out_6763671061317272364[55] = 0;
   out_6763671061317272364[56] = 0;
   out_6763671061317272364[57] = 0;
   out_6763671061317272364[58] = 0;
   out_6763671061317272364[59] = 0;
   out_6763671061317272364[60] = 1.0;
   out_6763671061317272364[61] = 0;
   out_6763671061317272364[62] = 0;
   out_6763671061317272364[63] = 0;
   out_6763671061317272364[64] = 0;
   out_6763671061317272364[65] = 0;
   out_6763671061317272364[66] = 0;
   out_6763671061317272364[67] = 0;
   out_6763671061317272364[68] = 0;
   out_6763671061317272364[69] = 0;
   out_6763671061317272364[70] = 1.0;
   out_6763671061317272364[71] = 0;
   out_6763671061317272364[72] = 0;
   out_6763671061317272364[73] = 0;
   out_6763671061317272364[74] = 0;
   out_6763671061317272364[75] = 0;
   out_6763671061317272364[76] = 0;
   out_6763671061317272364[77] = 0;
   out_6763671061317272364[78] = 0;
   out_6763671061317272364[79] = 0;
   out_6763671061317272364[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_990004811134166701) {
   out_990004811134166701[0] = state[0];
   out_990004811134166701[1] = state[1];
   out_990004811134166701[2] = state[2];
   out_990004811134166701[3] = state[3];
   out_990004811134166701[4] = state[4];
   out_990004811134166701[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_990004811134166701[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_990004811134166701[7] = state[7];
   out_990004811134166701[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6322414328777713969) {
   out_6322414328777713969[0] = 1;
   out_6322414328777713969[1] = 0;
   out_6322414328777713969[2] = 0;
   out_6322414328777713969[3] = 0;
   out_6322414328777713969[4] = 0;
   out_6322414328777713969[5] = 0;
   out_6322414328777713969[6] = 0;
   out_6322414328777713969[7] = 0;
   out_6322414328777713969[8] = 0;
   out_6322414328777713969[9] = 0;
   out_6322414328777713969[10] = 1;
   out_6322414328777713969[11] = 0;
   out_6322414328777713969[12] = 0;
   out_6322414328777713969[13] = 0;
   out_6322414328777713969[14] = 0;
   out_6322414328777713969[15] = 0;
   out_6322414328777713969[16] = 0;
   out_6322414328777713969[17] = 0;
   out_6322414328777713969[18] = 0;
   out_6322414328777713969[19] = 0;
   out_6322414328777713969[20] = 1;
   out_6322414328777713969[21] = 0;
   out_6322414328777713969[22] = 0;
   out_6322414328777713969[23] = 0;
   out_6322414328777713969[24] = 0;
   out_6322414328777713969[25] = 0;
   out_6322414328777713969[26] = 0;
   out_6322414328777713969[27] = 0;
   out_6322414328777713969[28] = 0;
   out_6322414328777713969[29] = 0;
   out_6322414328777713969[30] = 1;
   out_6322414328777713969[31] = 0;
   out_6322414328777713969[32] = 0;
   out_6322414328777713969[33] = 0;
   out_6322414328777713969[34] = 0;
   out_6322414328777713969[35] = 0;
   out_6322414328777713969[36] = 0;
   out_6322414328777713969[37] = 0;
   out_6322414328777713969[38] = 0;
   out_6322414328777713969[39] = 0;
   out_6322414328777713969[40] = 1;
   out_6322414328777713969[41] = 0;
   out_6322414328777713969[42] = 0;
   out_6322414328777713969[43] = 0;
   out_6322414328777713969[44] = 0;
   out_6322414328777713969[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6322414328777713969[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6322414328777713969[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6322414328777713969[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6322414328777713969[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6322414328777713969[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6322414328777713969[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6322414328777713969[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6322414328777713969[53] = -9.8000000000000007*dt;
   out_6322414328777713969[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6322414328777713969[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6322414328777713969[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6322414328777713969[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6322414328777713969[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6322414328777713969[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6322414328777713969[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6322414328777713969[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6322414328777713969[62] = 0;
   out_6322414328777713969[63] = 0;
   out_6322414328777713969[64] = 0;
   out_6322414328777713969[65] = 0;
   out_6322414328777713969[66] = 0;
   out_6322414328777713969[67] = 0;
   out_6322414328777713969[68] = 0;
   out_6322414328777713969[69] = 0;
   out_6322414328777713969[70] = 1;
   out_6322414328777713969[71] = 0;
   out_6322414328777713969[72] = 0;
   out_6322414328777713969[73] = 0;
   out_6322414328777713969[74] = 0;
   out_6322414328777713969[75] = 0;
   out_6322414328777713969[76] = 0;
   out_6322414328777713969[77] = 0;
   out_6322414328777713969[78] = 0;
   out_6322414328777713969[79] = 0;
   out_6322414328777713969[80] = 1;
}
void h_25(double *state, double *unused, double *out_3346788960512997050) {
   out_3346788960512997050[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4465233356863455756) {
   out_4465233356863455756[0] = 0;
   out_4465233356863455756[1] = 0;
   out_4465233356863455756[2] = 0;
   out_4465233356863455756[3] = 0;
   out_4465233356863455756[4] = 0;
   out_4465233356863455756[5] = 0;
   out_4465233356863455756[6] = 1;
   out_4465233356863455756[7] = 0;
   out_4465233356863455756[8] = 0;
}
void h_24(double *state, double *unused, double *out_92270251985139515) {
   out_92270251985139515[0] = state[4];
   out_92270251985139515[1] = state[5];
}
void H_24(double *state, double *unused, double *out_907307758061132620) {
   out_907307758061132620[0] = 0;
   out_907307758061132620[1] = 0;
   out_907307758061132620[2] = 0;
   out_907307758061132620[3] = 0;
   out_907307758061132620[4] = 1;
   out_907307758061132620[5] = 0;
   out_907307758061132620[6] = 0;
   out_907307758061132620[7] = 0;
   out_907307758061132620[8] = 0;
   out_907307758061132620[9] = 0;
   out_907307758061132620[10] = 0;
   out_907307758061132620[11] = 0;
   out_907307758061132620[12] = 0;
   out_907307758061132620[13] = 0;
   out_907307758061132620[14] = 1;
   out_907307758061132620[15] = 0;
   out_907307758061132620[16] = 0;
   out_907307758061132620[17] = 0;
}
void h_30(double *state, double *unused, double *out_1967819088175988157) {
   out_1967819088175988157[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2451456984628160999) {
   out_2451456984628160999[0] = 0;
   out_2451456984628160999[1] = 0;
   out_2451456984628160999[2] = 0;
   out_2451456984628160999[3] = 0;
   out_2451456984628160999[4] = 1;
   out_2451456984628160999[5] = 0;
   out_2451456984628160999[6] = 0;
   out_2451456984628160999[7] = 0;
   out_2451456984628160999[8] = 0;
}
void h_26(double *state, double *unused, double *out_3327446619185357985) {
   out_3327446619185357985[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8206736675737511980) {
   out_8206736675737511980[0] = 0;
   out_8206736675737511980[1] = 0;
   out_8206736675737511980[2] = 0;
   out_8206736675737511980[3] = 0;
   out_8206736675737511980[4] = 0;
   out_8206736675737511980[5] = 0;
   out_8206736675737511980[6] = 0;
   out_8206736675737511980[7] = 1;
   out_8206736675737511980[8] = 0;
}
void h_27(double *state, double *unused, double *out_4290298813414659485) {
   out_4290298813414659485[0] = state[3];
}
void H_27(double *state, double *unused, double *out_276693672827736088) {
   out_276693672827736088[0] = 0;
   out_276693672827736088[1] = 0;
   out_276693672827736088[2] = 0;
   out_276693672827736088[3] = 1;
   out_276693672827736088[4] = 0;
   out_276693672827736088[5] = 0;
   out_276693672827736088[6] = 0;
   out_276693672827736088[7] = 0;
   out_276693672827736088[8] = 0;
}
void h_29(double *state, double *unused, double *out_6969492355836833802) {
   out_6969492355836833802[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1436669054041814945) {
   out_1436669054041814945[0] = 0;
   out_1436669054041814945[1] = 1;
   out_1436669054041814945[2] = 0;
   out_1436669054041814945[3] = 0;
   out_1436669054041814945[4] = 0;
   out_1436669054041814945[5] = 0;
   out_1436669054041814945[6] = 0;
   out_1436669054041814945[7] = 0;
   out_1436669054041814945[8] = 0;
}
void h_28(double *state, double *unused, double *out_1352431057666297350) {
   out_1352431057666297350[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6519068071111345519) {
   out_6519068071111345519[0] = 1;
   out_6519068071111345519[1] = 0;
   out_6519068071111345519[2] = 0;
   out_6519068071111345519[3] = 0;
   out_6519068071111345519[4] = 0;
   out_6519068071111345519[5] = 0;
   out_6519068071111345519[6] = 0;
   out_6519068071111345519[7] = 0;
   out_6519068071111345519[8] = 0;
}
void h_31(double *state, double *unused, double *out_1944824502362540082) {
   out_1944824502362540082[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4434587394986495328) {
   out_4434587394986495328[0] = 0;
   out_4434587394986495328[1] = 0;
   out_4434587394986495328[2] = 0;
   out_4434587394986495328[3] = 0;
   out_4434587394986495328[4] = 0;
   out_4434587394986495328[5] = 0;
   out_4434587394986495328[6] = 0;
   out_4434587394986495328[7] = 0;
   out_4434587394986495328[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5829269461866699088) {
  err_fun(nom_x, delta_x, out_5829269461866699088);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2615413374656959367) {
  inv_err_fun(nom_x, true_x, out_2615413374656959367);
}
void car_H_mod_fun(double *state, double *out_6763671061317272364) {
  H_mod_fun(state, out_6763671061317272364);
}
void car_f_fun(double *state, double dt, double *out_990004811134166701) {
  f_fun(state,  dt, out_990004811134166701);
}
void car_F_fun(double *state, double dt, double *out_6322414328777713969) {
  F_fun(state,  dt, out_6322414328777713969);
}
void car_h_25(double *state, double *unused, double *out_3346788960512997050) {
  h_25(state, unused, out_3346788960512997050);
}
void car_H_25(double *state, double *unused, double *out_4465233356863455756) {
  H_25(state, unused, out_4465233356863455756);
}
void car_h_24(double *state, double *unused, double *out_92270251985139515) {
  h_24(state, unused, out_92270251985139515);
}
void car_H_24(double *state, double *unused, double *out_907307758061132620) {
  H_24(state, unused, out_907307758061132620);
}
void car_h_30(double *state, double *unused, double *out_1967819088175988157) {
  h_30(state, unused, out_1967819088175988157);
}
void car_H_30(double *state, double *unused, double *out_2451456984628160999) {
  H_30(state, unused, out_2451456984628160999);
}
void car_h_26(double *state, double *unused, double *out_3327446619185357985) {
  h_26(state, unused, out_3327446619185357985);
}
void car_H_26(double *state, double *unused, double *out_8206736675737511980) {
  H_26(state, unused, out_8206736675737511980);
}
void car_h_27(double *state, double *unused, double *out_4290298813414659485) {
  h_27(state, unused, out_4290298813414659485);
}
void car_H_27(double *state, double *unused, double *out_276693672827736088) {
  H_27(state, unused, out_276693672827736088);
}
void car_h_29(double *state, double *unused, double *out_6969492355836833802) {
  h_29(state, unused, out_6969492355836833802);
}
void car_H_29(double *state, double *unused, double *out_1436669054041814945) {
  H_29(state, unused, out_1436669054041814945);
}
void car_h_28(double *state, double *unused, double *out_1352431057666297350) {
  h_28(state, unused, out_1352431057666297350);
}
void car_H_28(double *state, double *unused, double *out_6519068071111345519) {
  H_28(state, unused, out_6519068071111345519);
}
void car_h_31(double *state, double *unused, double *out_1944824502362540082) {
  h_31(state, unused, out_1944824502362540082);
}
void car_H_31(double *state, double *unused, double *out_4434587394986495328) {
  H_31(state, unused, out_4434587394986495328);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
