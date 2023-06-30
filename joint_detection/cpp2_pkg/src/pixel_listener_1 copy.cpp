#include<ros/ros.h>
#include<ros/param.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/String.h>

#include <iostream>
#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

using namespace std;
using namespace Eigen;

Matrix3d K,M, K_inv;
Vector4d d;
VectorXd x(4), x_(4), x_theta(4), x_lambda(4);
MatrixXd Xc(3,5), Xc_(3,5), abc(5,5), Xw(3,4), Xw_(3,4), P(3,4), T(3,4);
float a1,a2,a3,a4,b1,b2,b3,b4,c1,c2,c3,c4,d1,d2,d3,d4;
float l1;
float a[2][2];

Matrix3d R, I, omega, Rot;
Vector3d t, v, trans;

Matrix4d M1, M2, M3, M4;
Matrix4d T1, T2, T3, T4;
MatrixXd es1(4,4),es2(4,4),es3(4,4),es4(4,4);
Matrix3d omega1, omega2, omega3, omega4;
Vector3d v1, v2, v3, v4;
Vector3d t1, t2, t3, t4;

//###############################################################################################################



void exp_m(MatrixXd &T,Matrix3d &omega, Vector3d &v,double theta)
{
    R = I + sin(theta)*omega + (1-cos(theta))*omega*omega;
    T.block(0,0,3,3) << R;
    t = (I*theta + (1-cos(theta))*omega + (theta-sin(theta))*omega*omega)*v;
    T.block(0,3,3,1) << t;
    T.row(3) << 0,0,0,1;
}

//###############################################################################################################

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor1
{
typedef _Scalar Scalar;
enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
};
typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

int m_inputs, m_values;

Functor1() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
Functor1(int inputs, int values) : m_inputs(inputs), m_values(values) {}

int inputs() const { return m_inputs; }
int values() const { return m_values; }

};

struct my_functor1 : Functor1<double>
{
my_functor1(void): Functor1<double>(4,4) {}
int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
    // Implement y = 10*(x0+3)^2 + (x1-5)^2
    //cout<<x<<endl;
    fvec(0) = d(0)*d(0) - (a1*l1*l1     + b1*l1*x(0)   + c1*x(0)*x(0));
    fvec(1) = d(1)*d(1) - (a2*x(0)*x(0) + b2*x(0)*x(1) + c2*x(1)*x(1));
    fvec(2) = d(2)*d(2) - (a3*x(1)*x(1) + b3*x(1)*x(2) + c3*x(2)*x(2));
    fvec(3) = d(3)*d(3) - (a4*x(2)*x(2) + b4*x(2)*x(3) + c4*x(3)*x(3));
    //cout<<fvec<<endl;
    return 0;
}
};

//###############################################################################################################

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor2
{
typedef _Scalar Scalar;
enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
};
typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

int m_inputs, m_values;

Functor2() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
Functor2(int inputs, int values) : m_inputs(inputs), m_values(values) {}

int inputs() const { return m_inputs; }
int values() const { return m_values; }

};

struct my_functor2 : Functor2<double>
{
my_functor2(void): Functor2<double>(4,4) {}
int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
    //cout<<"%%%%%%%%%%%%%%%%%%%%     1      %%%%%%%%%%%%%%%%%%%%%%%%"<<endl<<endl;
    exp_m(es1,omega1,v1,x(0));
    exp_m(es2,omega2,v2,x(1));
    exp_m(es3,omega3,v3,x(2));
    exp_m(es4,omega4,v4,x(3));

    //cout<<"%%%%%%%%%%%%%%%%%%%%     2      %%%%%%%%%%%%%%%%%%%%%%%%"<<endl<<endl;
    T1 = es1*M1;
    T2 = es1*es2*M2;
    T3 = es1*es2*es3*M3;
    T4 = es1*es2*es3*es4*M4;

    //cout<<"%%%%%%%%%%%%%%%%%%%%     3      %%%%%%%%%%%%%%%%%%%%%%%%"<<endl<<endl;

    t1 << T1.block(0,3,3,1);
    t2 << T2.block(0,3,3,1);
    t3 << T3.block(0,3,3,1);
    t4 << T4.block(0,3,3,1);

    //cout<<"%%%%%%%%%%%%%%%%%%%%     4      %%%%%%%%%%%%%%%%%%%%%%%%"<<endl<<endl;

    Xw.block(0,0,3,1) << t1;
    Xw.block(0,1,3,1) << t2;
    Xw.block(0,2,3,1) << t3;
    Xw.block(0,3,3,1) << t4;

    //cout<<"%%%%%%%%%%%%%%%%%%%%     5      %%%%%%%%%%%%%%%%%%%%%%%%"<<endl<<endl;

    //cout<<Xw<<endl<<endl;
    //cout<<T1<<endl<<endl;
    //cout<<T2<<endl<<endl;
    //cout<<T3<<endl<<endl;
    //cout<<T4<<endl<<endl;

    Vector3d temp;
    temp << Xw.col(0)-Xw_.col(0);
    //cout<<temp.norm()<<endl<<endl;
    fvec(0) = temp.norm();
    temp << Xw.col(1)-Xw_.col(1);
    fvec(1)=temp.norm();
    temp << Xw.col(2)-Xw_.col(2);
    fvec(2)=temp.norm();
    temp << Xw.col(3)-Xw_.col(3);
    fvec(3)=temp.norm();

    //cout<<"fvec :"<<fvec<<endl<<endl;
    return 0;
}
};

//###############################################################################################################

void sub_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

    Xc << msg->data[0],msg->data[3],msg->data[6],msg->data[9],msg->data[12],
          msg->data[1],msg->data[4],msg->data[7],msg->data[10],msg->data[13],
          msg->data[2],msg->data[5],msg->data[8],msg->data[11],msg->data[14];

    Xc << 875, 858, 754, 623, 527,
    579, 490, 510, 511, 509,
    1.0000,    1.0000,    1.0000,    1.0000,    1.0000;
    
    abc = Xc.transpose().eval() * M * Xc;
    a1 = abc(0,0);
    a2 = abc(1,1);
    a3 = abc(2,2);
    a4 = abc(3,3);

    b1 = -(abc(0,1) + abc(1,0));
    b2 = -(abc(1,2) + abc(2,1));
    b3 = -(abc(2,3) + abc(3,2));
    b4 = -(abc(3,4) + abc(4,3));

    c1 = a2;
    c2 = a3;
    c3 = a4;
    c4 = abc(4,4);

    d << 0.2790163480873476,
         0.37089477807063936,
         0.3744235624263116,
         0.22952499999999992;

    l1 = 3-0.063186;   // Known value: depth of fixed point from camera
    //________________________________________________________________________________
    my_functor1 functor1;
    Eigen::NumericalDiff<my_functor1> numDiff(functor1);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor1>,double> lm(numDiff);
    lm.parameters.maxfev = 500;
    lm.parameters.xtol = 1.0e-10;
    //std::cout << lm.parameters.maxfev << std::endl;
    
    x << x_lambda;
    int ret = lm.minimize(x);
    x_lambda << x;
    std::cout << "x_lambda that minimizes the function: " << x_lambda << std::endl;
    //std::cout << lm.iter << std::endl;
    //std::cout << ret << std::endl;

    //std::cout << "x lambda that minimizes the function: " << x << std::endl;

    Xw_.col(0) << trans + x(0)*Rot*K_inv*Xc.col(1);
    Xw_.col(1) << trans + x(1)*Rot*K_inv*Xc.col(2);
    Xw_.col(2) << trans + x(2)*Rot*K_inv*Xc.col(3);
    Xw_.col(3) << trans + x(3)*Rot*K_inv*Xc.col(4);
    cout<<"Xw : "<<Xw_<<endl<<endl;


    //________________________________________________________________________________
    my_functor2 functor2;
    //cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"<<endl;
    Eigen::NumericalDiff<my_functor2> numDiff2(functor2);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor2>,double> lm2(numDiff2);
    lm2.parameters.maxfev = 500;
    lm2.parameters.xtol = 1.0e-10;
    //std::cout << lm2.parameters.maxfev << std::endl;

    x << x_theta;
    ret = lm2.minimize(x);
    x_theta << x;
    //std::cout << lm2.iter << std::endl;
    //std::cout << ret << std::endl;

    std::cout << "x_ that minimizes the function: " << x_theta << std::endl;
}

//###############################################################################################################

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"pixel_listener_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("keypoint_pixels",1000,sub_callback);
    ros::Rate rate(1);

    Eigen::VectorXd x(4);
    x.setZero();
    std::cout << "x: " << x << std::endl;

    x_theta.setZero();
    x_lambda.setZero();
    //______________________________________________________
    // Parameters of baxter robot

    omega1 << 0  , -1.0000 ,  -0.0000,
        1.0000   ,      0  ,  0.0000,
        0.0000   ,-0.0000  ,      0;

    omega2 << 0   ,-0.0000    ,0.7071,
        0.0000         ,0   ,-0.7071,
       -0.7071    ,0.7071         ,0;

    omega3 << 0   ,-0.0000    ,0.7071,
        0.0000    ,    0   ,-0.7071,
       -0.7071    ,0.7071         ,0;

    omega4 << 0   ,-0.0000    ,0.7071,
        0.0000    ,     0   ,-0.7071,
       -0.7071    ,0.7071   ,      0;

    M1 << 0.7071,    0.0000,    0.7071,    0.1128,
       -0.7071,    0.0000,    0.7071,   -0.3078,
             0,   -1.0000,    0.0000,    0.4000,
             0,         0,         0,    1.0000;

    M2 << 0.7071,    0.0000,    0.7071,    0.3705,
       -0.7071,    0.0000,    0.7071,   -0.5655,
        0.0000,   -1.0000,    0.0000,    0.3310,
             0,         0,        0 ,   1.0000;

    M3 << 0.7071,    0.0000,    0.7071,   0.6352,
       -0.7071,    0.0000,    0.7071,   -0.8302,
        0.0000,   -1.0000,    0.0000,    0.3210,
             0,         0,         0,    1.0000;

    M4 << 0.0000,    0.7071,    0.7071,    0.7975,
        0.0000,    0.7071,   -0.7071,   -0.9925,
       -1.0000,    0.0000,    0.0000,    0.3210,
             0,         0,         0,   1.0000;

    v1 << -0.2590,
       -0.0640,
        0.0000;

    v2 << -0.2828,
        0.2828,
        0.2974;

    v3 << -0.2340,
        0.2340,
        0.6619;

    v4 << -0.2270,
        0.2270,
        1.0361;

    //______________________________________________________
    I << 1,0,0,
    0,1,0,
    0,0,1;

    K << 918.6685791  ,  0     ,    639.38970947,
         0     ,    918.41821289, 361.01498413,
         0     ,      0         ,  1 ;       

    K_inv = K.inverse();

    Rot <<  0,0,-1,
    1,0,0,
    0,-1,0;

    trans << 3,
    0,
    0.25;

    cout<<trans<<endl;

    M = K*K.transpose();
    M = M.inverse().eval();

    Xc << 875, 858, 753, 630, 589,
579, 490, 461, 498, 582,
1, 1, 1, 1, 1;

    ros::spin();
    return 0;
}
