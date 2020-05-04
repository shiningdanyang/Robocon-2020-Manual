#include "inverteKinematic.h"
void solve_IK()
{
    uB = sB * sqrt(3) / 3;
    wB = sB * sqrt(3) / 6;
    uP = sP * sqrt(3) / 3;
    wP = sP * sqrt(3) / 6;

    Ob1[_x] = sB / 2;
    Ob1[_y] = -wB;
    Ob1[_z] = 0;

    Ob2[_x] = 0;
    Ob2[_y] = uB;
    Ob2[_z] = 0;

    Ob3[_x] = -sB / 2;
    Ob3[_y] = -wB;
    Ob3[_z] = 0;

    OB1[_x] = 0;
    OB1[_y] = -wB;
    OB1[_z] = 0;

    OB2[_x] = sqrt(3) * wB / 2;
    OB2[_y] = wB / 2;
    OB2[_z] = 0;

    OB3[_x] = -sqrt(3) * wB / 2;
    OB3[_y] = wB / 2;
    OB3[_z] = 0;

    A1[_x] = 0;
    A1[_y] = L - wB;
    A1[_z] = 0;
    A2[_x] = (sqrt(3) / 2) * (wB - L);
    A2[_y] = (1 / 2) * (wB - L);
    A2[_z] = 0;
    A3[_x] = (sqrt(3) / 2) * (L - wB);
    A3[_y] = (1 / 2) * (L + wB);
    A3[_z] = 0;

    //assign_vector3D(planeA1_normal, OB1);
    //assign_vector3D(planeA2_normal, OB2);
    //assign_vector3D(planeA3_normal, OB3);

    //dotProduct(planeA1_d, planeA1_normal, A1);
    //dotProduct(planeA2_d, planeA2_normal, A2);
    //dotProduct(planeA3_d, planeA3_normal, A3);
    //planeA1_d = -planeA1_d;
    //planeA2_d = -planeA2_d;
    //planeA3_d = -planeA3_d;

    OP1[_x] = x + 0;
    OP1[_y] = y + -uP;
    OP1[_z] = z + 0;

    OP2[_x] = x + sP / 2;
    OP2[_y] = y + wP;
    OP2[_z] = z + 0;

    OP3[_x] = x + -sP / 2;
    OP3[_y] = y + wP;
    OP3[_z] = z + 0;

    //sub_vector3D(b3P1, OP1, Ob3);
    //sub_vector3D(b1P1, OP1, Ob1);
    //sub_vector3D(b1P2, OP2, Ob1);
    //sub_vector3D(b2P2, OP2, Ob2);
    //sub_vector3D(b2P3, OP3, Ob2);
    //sub_vector3D(b3P3, OP3, Ob3);

    //crossProduct(planeP1_normal, b3P1, b1P1);
    //crossProduct(planeP2_normal, b1P2, b2P2);
    //crossProduct(planeP3_normal, b2P3, b3P3);

    //dotProduct(planeP1_D, planeP1_normal, OP1);
    //planeP1_D = -planeP1_D;
    //dotProduct(planeP2_D, planeP2_normal, OP2);
    //planeP2_D = -planeP2_D;
    //dotProduct(planeP3_D, planeP3_normal, OP3);
    //planeP3_D = -planeP3_D;

    E1 = 2 * L * (y + a);
    F1 = 2 * z * L;
    G1 = x * x + y * y + z * z + a * a + L * L + 2 * y * a - l * l;
    E2 = -L * ((x + b) * sqrt(3) + y + c);
    F2 = 2 * z * L;
    G2 = x * x + y * y + z * z + b * b + c * c + L * L + 2 * (x * b + y * c) - l * l;
    E3 = L * ((x - b) * sqrt(3) - y - c);
    F3 = 2 * z * L;
    G3 = x * x + y * y + z * z + b * b + c * c + L * L + 2 * (-x * b + y * c) - l * l;

    a = -uP + wB;
    b = sP / 2 - wB * sqrt(3) / 2;
    c = wP - wB / 2;

    t[0][n1] = ((-F1 - sqrt(E1 * E1 + F1 * F1 - G1 * G1)) / (G1 - E1));
    q[0][n1] = 2 * atan(t[0][n1]);
    t[0][n2] = ((-F1 + sqrt(E1 * E1 + F1 * F1 - G1 * G1)) / (G1 - E1));
    q[0][n2] = 2 * atan(t[0][n2]);
    if (q[0][n1] < q[0][n2])
        q1_res = q[0][n1];
    else
        q1_res = q[0][n2];

    t[1][n1] = ((-F2 - sqrt(E2 * E2 + F2 * F2 - G2 * G2)) / (G2 - E2));
    q[1][n1] = 2 * atan(t[1][n1]);
    t[1][n2] = ((-F2 + sqrt(E2 * E2 + F2 * F2 - G2 * G2)) / (G2 - E2));
    q[1][n2] = 2 * atan(t[1][n2]);
    if (q[1][n1] < q[1][n2])
        q2_res = q[1][n1];
    else
        q2_res = q[1][n2];

    t[2][n1] = ((-F3 - sqrt(E3 * E3 + F3 * F3 - G3 * G3)) / (G3 - E3));
    q[2][n1] = 2 * atan(t[2][n1]);
    t[2][n2] = ((-F3 + sqrt(E3 * E3 + F3 * F3 - G3 * G3)) / (G3 - E3));
    q[2][n2] = 2 * atan(t[2][n2]);
    if (q[2][n1] < q[2][n2])
        q3_res = q[2][n1];
    else
        q3_res = q[2][n2];

}