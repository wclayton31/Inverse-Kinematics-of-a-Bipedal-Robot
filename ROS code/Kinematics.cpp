/*
* William Clayton
*/

//Base Libraries
#include <iostream>
#include <cmath>
#include <sstream>

//Linear Algebra Libraries (Armadillo)
#include </usr/include/armadillo>

//Ros Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace arma;
using namespace ros;

double pi = (atan(1.0) * 4);
double e = .000001;
//DH parameters: 1     2     3     4     5     6     7     8     9    10
double a[10] = {.5,    1,   .5,   .5,    0,   .5,    1,   .5,   .5,   0},
       w[10] = {pi/2,  0,    0,  -pi/2, pi/2, pi/2,  0,    0,  -pi/2,pi/2},
       d[10] = {0,     0,    0,    0,    0,    0,    0,    0,    0,   0},
       v[10];

//Direct Kinematics matrices
mat t_r(4,4), t_l(4,4), ab_r(4,4), ab_l(4,4), a_r[5], a_l[5];

mat id(4,4);

void send_set(int*, Publisher);
void initialize_posiiton(mat*,mat*);
void set_dk(double*);   //set direct kinematics
void set_dk(double*, bool);
mat homogen_matrix(double,double,double,double);
void get_inverse(mat, mat, double*);
void convert_joint_angles(double*,int*);

int steps = 3;

int main(int argc, char **argv)
{
    id = eye<mat>(4,4);
    ab_r = homogen_matrix(.5, -1*pi/2, 0, 0);
    ab_l = homogen_matrix(-.5, -1*pi/2, 0, 0);
    //Ros variable:
    init(argc, argv, "Kinematics");        //Name of the ros Node
    NodeHandle n;   //ros node

    Publisher position_pub = n.advertise<std_msgs::String>("kinematics", 1000);   //publisher name

    char yes;
    cout << "Ready?" << endl;
    std::cin >> yes;

    double v[10] = {0 + pi/2.0, 0,0,0,0 - pi/2.0,       //initialized state
                    0 + pi/2.0,0,0,0,0 - pi/2.0};
    steps = 6;
    mat p_r[6], p_l[6];
    initialize_posiiton(p_r,p_l);
    double x = .6;//y = .5;
    //Standing position:
    //**********************************************
    p_r[0](0,3) = .5;
    p_r[0](2,3) = -2.5;
    p_l[0](0,3) = -.5;
    p_l[0](2,3) = -2.5;
    //**********************************************
    p_r[1](0,3) = .5 - x;
    p_r[1](1,3) = 0;
    p_r[1](2,3) = -sqrt(pow(2.5,2) - pow(x,2));
    p_l[1](0,3) = -.5 - x;
    p_l[1](2,3) = -sqrt(pow(2.5,2) - pow(x,2));
    //***********************************************
    p_r[2](0,3) = .2;
    p_r[2](1,3) = 1;
    p_r[2](2,3) = -1.4697;
    p_l[2](0,3) = -.5 - x;
    p_l[2](2,3) = -sqrt(pow(2.5,2) - pow(x,2));
    //**********************************************
    p_r[3](0,3) = .5;
    p_r[3](1,3) = .50853;
    p_r[3](2,3) = -2.166;

    p_l[3](0,3) = -.5;
    p_l[3](1,3) = -.76659;
    p_l[3](2,3) = -2.2001;
    //**********************************************
    p_r[4](0,3) = .5+x;
    p_r[4](1,3) = 0;
    p_r[4](2,3) = -sqrt(pow(2.5,2) - pow(x,2));
    p_l[4](0,3) = -.3;
    p_l[4](1,3) = 1;
    p_l[4](2,3) = -1.4697;
    //**********************************************/
    p_r[5](0,3) = .5;
    p_r[5](1,3) = -.76659;
    p_r[5](2,3) = -2.2001;

    p_l[5](0,3) = -.5;
    p_l[5](1,3) = .50853;
    p_l[5](2,3) = -2.166;

    for(int i=0; i<steps; i++)
    {
        //cout << "**************************************************" << endl;
        cout << "Step: " << i << endl;
        get_inverse(p_r[i],p_l[i], v);

        int out[10];
        convert_joint_angles(v,out);
        //for(int j=0; j<10; j++)
            //cout << i << " Step\t"<< j << ": Angle: " << v[j]*180/pi << "\tout: " << out[j] << endl;

        send_set(out, position_pub);
        //cout << "****************************************************" << endl;
        //cout << endl;
        sleep(2);
    }

    double standing[10] = {0,0,0,0,0,0,0,0,0,0};
    int s_output[10];
    convert_joint_angles(standing, s_output);
    send_set(s_output, position_pub);

    spin();
    return 0;
}

void send_set(int* v, Publisher position_pub)
{
    std::stringstream ss;
    for(int i=0; i<10; i++)
        ss << "#" << i << "P" << static_cast<int>(v[i]);
    ss << "T2000\n";

    std_msgs::String data;
    data.data = ss.str();
    cout << ss.str() << endl;
    position_pub.publish(data);
}

void initialize_posiiton(mat* p_r, mat* p_l)
{
    for(int i=0; i<steps; i++)
    {
        p_r[i].set_size(4,4);
        p_r[i] = id;
        p_l[i].set_size(4,4);
        p_l[i] = id;
    }
}

void convert_joint_angles(double* v, int* out)
{
    double pulse_to_radians = 2000/pi;     //=pulse/radians
    out[0] = static_cast<int>(1560 + v[0]*pulse_to_radians);
    out[1] = static_cast<int>(1510 - v[1]*pulse_to_radians);
    out[2] = static_cast<int>(1540 + v[2]*pulse_to_radians);
    out[3] = static_cast<int>(1480 + v[3]*pulse_to_radians);
    out[4] = static_cast<int>(1540 - v[4]*pulse_to_radians);

    out[5] = static_cast<int>(1790 + v[5]*pulse_to_radians);
    out[6] = static_cast<int>(1510 + v[6]*pulse_to_radians);
    out[7] = static_cast<int>(1450 - v[7]*pulse_to_radians);
    out[8] = static_cast<int>(1460 - v[8]*pulse_to_radians);
    out[9] = static_cast<int>(1540 - v[9]*pulse_to_radians);

    out[0] = (out[0]< 1350?1350:(out[0]>2100?2100:out[0]));
    out[1] = (out[1]< 800?800:(out[1]>1950?1950:out[1]));
    out[2] = (out[2]< 800?800:(out[2]>1950?1950:out[2]));
    out[3] = (out[3]< 1100?1100:(out[3]>2200?2200:out[3]));
    out[4] = (out[4]< 1150?1150:(out[4]>2200?2200:out[4]));

    out[5] = (out[5]< 1250?1250:(out[5]>1950?1950:out[5]));
    out[6] = (out[6]< 1150?1150:(out[6]>2200?2200:out[6]));
    out[7] = (out[7]< 1050?1050:(out[7]>2200?2200:out[7]));
    out[8] = (out[8]< 800?800:(out[8]>1850?1850:out[8]));
    out[9] = (out[9]< 800?800:(out[9]>1950?1950:out[9]));
}

void get_inverse(mat TR, mat TL, double* v)
{
    vec q_r = TR.col(3), q_l = TL.col(3);
    //Finding v1 from the given q
    // v1 = arctan(x/z)                        hip is set to pi/2 max to be able to find positions for the foot
    v[0] = atan2(-1*(q_r(0)-.5),-1*q_r(2));                 //above the hip
    v[5] = atan2(-1*(q_l(0)+.5),-1*q_l(2));
    v[0] = (e>q_r(2) && q_r(2)>-1*e?0:v[0]);
    v[0] -= (e>q_r(0)-.5 && q_r(0)-.5>-1*e && q_r(2)>0?pi:0);
    v[5] = (e>q_l(2) && q_l(2)>-1*e?0:v[5]);
    v[5] -= (e>q_l(0)-.5 && q_l(0)-.5>-1*e && q_l(2)>0?pi:0);

    //*************************************************************
    //(ab_r*ar[0])^-1 * TR = ar[1]*ar[2]*ar[3]*ar[4]
    //calculate the foot position/oreintation with respect to the second hip joint
    mat abr1_inv(4,4), abl1_inv(4,4);
    abr1_inv = homogen_matrix(a[0],w[0],d[0],v[0]+pi/2);
    abr1_inv = ab_r * abr1_inv;
    abr1_inv = inv(abr1_inv);
    abl1_inv = homogen_matrix(a[0],w[0],d[0],v[0]+pi/2);
    abl1_inv = ab_l * abl1_inv;
    abl1_inv = inv(abl1_inv);

    mat abr_t(4,4), abl_t(4,4);     //abr_t = ar[1]*ar[2]*ar[3]*ar[4]
    abr_t = abr1_inv * TR;
    abl_t = abl1_inv * TL;

    //**************************************************************
    vec qh_r = abr_t.col(3), qh_l = abl_t.col(3);  //the fourth col is the position of the point, the last frame ar[4], does not affect the position
    vec sh_r = abr_t.col(1), sh_l = abl_t.col(1);  //the second col is the rotation about the y axis, abr_t falls entirely in the x-y plane,
                                                   //the rotation of the final point shows the end amount of rotation of the leg

    double phi_r = atan2(-1*sh_r(0),sh_r(1)), phi_l = atan2(-1*sh_l(0),sh_l(1));    //finding the total rotation of the leg

    vec qw_r(3), qw_l(3);       //marks the point of the anlke
    qw_r(0) =  qh_r(0) - a[3]*cos(phi_r);
    qw_r(1) =  qh_r(1) - a[3]*sin(phi_r);
    qw_l(0) = qh_l(0) - a[8]*cos(phi_l);
    qw_l(1) = qh_l(1) - a[8]*sin(phi_l);

    //uses the formula derived: c3 = (qw^2x + qwy^2 - a[1]^2 - a[2]^2)/2a[1]a[2]
    //treating A2-5 as a planar robot since A5 does not change position, and only affects rotation around the y axis

    double cosV3_r = (pow(qw_r(0),2) + pow(qw_r(1),2) - pow(a[1],2) - pow(a[2],2))/(2.0*a[1]*a[2]),
           sinV3_r = -sqrt(1-pow(cosV3_r, 2));
    double cosV3_l = (pow(qw_l(0),2) + pow(qw_l(1),2) - pow(a[6],2) - pow(a[7],2))/(2.0*a[6]*a[7]),
           sinV3_l = -sqrt(1-pow(cosV3_l, 2));

    v[2] = atan2(sinV3_r,cosV3_r);
    v[7] = atan2(sinV3_l,cosV3_l);


    double sinV2_r = ((a[1] + a[2]*cos(v[2]))*qw_r(1) - a[2]*sin(v[2])*qw_r(0))/(pow(qw_r(0),2) + pow(qw_r(1),2)),
           cosV2_r = ((a[1] + a[2]*cos(v[2]))*qw_r(0) + a[2]*sin(v[2])*qw_r(1))/(pow(qw_r(0),2) + pow(qw_r(1),2));
    v[1] = atan2(sinV2_r, cosV2_r);

    double sinV2_l = ((a[6] + a[7]*cos(v[7]))*qw_l(1) - a[7]*sin(v[7])*qw_l(0))/(pow(qw_l(0),2) + pow(qw_l(1),2)),
           cosV2_l = ((a[6] + a[7]*cos(v[7]))*qw_l(0) + a[7]*sin(v[7])*qw_l(1))/(pow(qw_l(0),2) + pow(qw_l(1),2));
    v[6] = atan2(sinV2_l, cosV2_l);
    //finds the final ankle rotation by subtracting the two other found rotations
    //from the total rotation of the leg
    v[3] = phi_r - v[1] - v[2];
    v[8] = phi_l - v[6] - v[7];

    //creates the leg matrices to take the inv and multiply to the final point
    mat a2_r(4,4), a3_r(4,4), a4_r(4,4), a2_l(4,4), a3_l(4,4), a4_l(4,4);
    a2_r = homogen_matrix(a[1],w[1],d[1],v[1]);
    a3_r = homogen_matrix(a[2],w[2],d[2],v[2]);
    a4_r = homogen_matrix(a[3],w[3],d[3],v[3]);
    a2_l = homogen_matrix(a[6],w[6],d[6],v[6]);
    a3_l = homogen_matrix(a[7],w[7],d[7],v[7]);
    a4_l = homogen_matrix(a[8],w[8],d[8],v[8]);

    mat a5_r = inv(a4_r)*inv(a3_r)*inv(a2_r)*abr1_inv*TR, a5_l = inv(a4_l)*inv(a3_l)*inv(a2_l)*abl1_inv*TL;
    //then finds the final rotation of the ankle about the y
    vec n_r = a5_r.col(0), n_l = a5_l.col(0);
    v[4] = atan2(n_r(0),-1*n_r(1));
    v[9] = atan2(n_l(0), -1*n_l(1));
}

void set_dk(double v[10], bool adjust)
{
    if(adjust)
    {
        v[0]+=pi/2;
        v[4]-=pi/2;
        v[5]+=pi/2;
        v[9]-=pi/2;
    }
    set_dk(v);
}

void set_dk(double v[10])
{
    for(int i=0; i<5; i++)
    {
        a_r[i].set_size(4,4);
        a_r[i] = homogen_matrix(a[i], w[i], d[i], v[i]);
        a_l[i].set_size(4,4);
        a_l[i] = homogen_matrix(a[i+5], w[i+5], d[i+5], v[i+5]);
    }

    /*DEBUG********************************************
    mat r_link1(4,4), r_link2(4,4), r_link3(4,4), r_link4(4,4);
    r_link1 = ab_r*a_r[0];
    r_link2 = ab_r*a_r[0]*a_r[1];
    r_link3 = ab_r*a_r[0]*a_r[1]*a_r[2];
    r_link4 = ab_r*a_r[0]*a_r[1]*a_r[2]*a_r[3];

    r_link1.print("R Link1:");
    r_link2.print("R Link2:");
    r_link3.print("R Link3:");
    r_link4.print("R Link4:");

    mat l_link1(4,4), l_link2(4,4), l_link3(4,4), l_link4(4,4);
    l_link1 = ab_l*a_l[0];
    l_link2 = ab_l*a_l[0]*a_l[1];
    l_link3 = ab_l*a_l[0]*a_l[1]*a_l[2];
    l_link4 = ab_l*a_l[0]*a_l[1]*a_l[2]*a_l[3];

    l_link1.print("L Link1:");
    l_link2.print("L Link2:");
    l_link3.print("L Link3:");
    l_link4.print("L Link4:");  //************************/

    t_r = ab_r*a_r[0]*a_r[1]*a_r[2]*a_r[3]*a_r[4];
    t_l = ab_l*a_l[0]*a_l[1]*a_l[2]*a_l[3]*a_l[4];

    t_r.print("TR:");
    t_l.print("TL:");
}

mat homogen_matrix(double ai, double wi, double di, double vi)
{
    mat A(4,4);
    A << cos(vi) << -1*sin(vi)*cos(wi) << sin(vi)*sin(wi)    << ai*cos(vi) << endr
      << sin(vi) << cos(vi)*cos(wi)    << -1*cos(vi)*sin(wi) << ai*sin(vi) << endr
      << 0       << sin(wi)            << cos(wi)            << di         << endr
      << 0       << 0                  << 0                  << 1          << endr;
    return A;
}
