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
int i;

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


//____________________________________________________________________________________________

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
my_functor1(void): Functor1<double>(4,8) {}
int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
    // Implement y = 10*(x0+3)^2 + (x1-5)^2
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

    //cout<<Xw<<endl;
    //cout<<"%%%%%%%%%%%%%%%%%%%%     5      %%%%%%%%%%%%%%%%%%%%%%%%"<<endl<<endl;

    Xc.col(1) = K*Rot.transpose()*(Xw.col(0) - trans);
    Xc.col(1) = Xc.col(1)/Xc(2,1);

    Xc.col(2) = K*Rot.transpose()*(Xw.col(1) - trans);
    Xc.col(2) = Xc.col(2)/Xc(2,2);

    Xc.col(3) = K*Rot.transpose()*(Xw.col(2) - trans);
    Xc.col(3) = Xc.col(3)/Xc(2,3);

    Xc.col(4) = K*Rot.transpose()*(Xw.col(3) - trans);
    Xc.col(4) = Xc.col(4)/Xc(2,4);

    cout<<Xc.block(0,1,2,4)<<endl<<endl;
    cout<<Xc_.block(0,1,2,4)<<endl<<endl;

    Vector3d temp;
    temp << Xc_.col(1)-Xc.col(1);
    fvec(0) = temp(0);
    fvec(1) = temp(1);

    temp << Xc.col(2)-Xc_.col(2);
    fvec(2) = temp(0);
    fvec(3) = temp(1);

    temp << Xc.col(3)-Xc_.col(3);
    fvec(4) = temp(0);
    fvec(5) = temp(1);

    temp << Xc.col(4)-Xc_.col(4);
    fvec(6) = temp(0);
    fvec(7) = temp(1);

    std::cout << "x: " << x.transpose() << std::endl;
    std::cout << "fvec: " << fvec.transpose() << std::endl << std::endl;
    //my_file <<x(0)<<" "<<x(1)<<" "<<x(2)<<" "<<x(3)<<"\n";
    cout<<"__________________________"<<i<<"_____________________________"<<std::endl;
    i = i+1;
    return 0;
}
};


//###############################################################################################################

void sub_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    Xc_ << msg->data[0],msg->data[2],msg->data[4],msg->data[6],msg->data[8],
          msg->data[1],msg->data[3],msg->data[5],msg->data[7],msg->data[9],
          1,1,1,1,1;
    
    std::cout<<"___________________"<<Xc_<<std::endl;

    my_functor1 functor1;
    Eigen::NumericalDiff<my_functor1> numDiff(functor1);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor1>,double> lm(numDiff);
    lm.parameters.maxfev = 100;
    lm.parameters.xtol = 1.0e-10;
    //lm.minimize(x);
    std::cout << "x: " << x.transpose() << std::endl;
    i=0;
}

//###############################################################################################################

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"pixel_listener_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/dream/keypoint",1000,sub_callback);
    ros::Rate rate(1);

    x_theta.setZero();
    x_lambda.setZero();
    //______________________________________________________
    // Parameters of baxter robot

    i = 0;
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

    I << 1,0,0,
    0,1,0,
    0,0,1;

    K << 918.6685791  ,  0     ,    639.38970947,
         0     ,    918.41821289, 361.01498413,
         0     ,      0         ,  1 ;

    K_inv = K.inverse();

    Rot <<  0.03317783105089033, -0.06928384585727625, -0.9968911764175806,
            0.9992971260833398, 0.008394440575496175, 0.0326725612797906,
            0.006106325171848064, -0.9975410329582914, 0.06953223576690637;

    trans <<  2.0950551217101343,
             -0.13581018332957323,
              0.18659907817209642;

    x.setConstant(0);
    std::cout << "x: " << x.transpose() << std::endl;

    ros::spin();
    sub.shutdown();

    my_functor1 functor1;
    Eigen::NumericalDiff<my_functor1> numDiff(functor1);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor1>,double> lm(numDiff);
    lm.parameters.maxfev = 200;
    lm.parameters.xtol = 1.0e-10;
    lm.parameters.factor = 0.1;
    lm.minimize(x);
    std::cout<<"Factor : "<<lm.parameters.factor<<std::endl;
    std::cout << "x: " << x.transpose() << std::endl;
    std::cout << "Xc: " << Xc_ << std::endl;
    i=0;

    return 0;
}
