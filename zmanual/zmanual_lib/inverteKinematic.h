#include <stdlib.h>
#include <math.h>
#define _x 0
#define _y 1
#define _z 2
#define n1 0
#define n2 1

double x = 0, y = 0, z = -0.9;
double q1_res, q2_res, q3_res;
const double L = 0.3;
const double l = 0.8;
const double sB = 0.64;
const double sP = 0.1;

double uB, wB, uP, wP;

double Ob1[3];
double Ob2[3];
double Ob3[3];

double OB1[3];
double OB2[3];
double OB3[3];

double A1[3];
double A2[3];
double A3[3];

double OP1[3];
double OP2[3];
double OP3[3];

double b3P1[3];
double b1P1[3];
double b1P2[3];
double b2P2[3];
double b2P3[3];
double b3P3[3];

double a, b, c;
double E1, F1, G1;
double E2, F2, G2;
double E3, F3, G3;

double t[3][2];
double q[3][2];

double B1A1_n1[3];
double B1A1_n2[3];
double A1_n1[3];
double A1_n2[3];

double B2A2_n1[3];
double B2A2_n2[3];
double A2_n1[3];
double A2_n2[3];

double B3A3_n1[3];
double B3A3_n2[3];
double A3_n1[3];
double A3_n2[3];

void solve_IK();