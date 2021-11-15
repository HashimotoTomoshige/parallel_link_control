/**********************************************************************\
 * [目的]: 
 * 目標姿勢から、球面パラレルリンク(Arduino)に対する指令値とリニアモータに対する指令値を計算するプログラム
 * [諸条件]:
 * 
 * [コーディングに関して注意事項]
 * Eigenとstd::vectorでは四則演算は不可能
 * Eigen::VectorXdとEigen::MatrixXdは初期化しないと使えない
 * 
\**********************************************************************/

#include "parallel_link_control/compute_kinematics.h"

namespace parallel_link_control
{
    ComputeKinematics::ComputeKinematics()
    {
        //write_flag_ = true;
    }

    Eigen::VectorXd ComputeKinematics::computePosture(Eigen::VectorXd r_goal) //目標の手先位置姿勢ベクトルから関節変位ベクトルを算出する関数
    {
        Eigen::VectorXd r_p = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd bryant_angle = Eigen::VectorXd::Zero(3);
        bryant_angle << r_goal(3), r_goal(4), r_goal(5);
        Eigen::VectorXd zxy_fixed_angle = transformAngleExpression(bryant_angle);

        r_p(0) = 0;
        r_p(1) = r_goal(1);
        r_p(2) = r_goal(2);
        r_p(3) = zxy_fixed_angle(0);
        r_p(4) = zxy_fixed_angle(1);
        r_p(5) = zxy_fixed_angle(2);
        
        return r_p;
    }

    Eigen::VectorXd ComputeKinematics::transformAngleExpression(Eigen::VectorXd& bryant_angle) //bryant_angle = [roll, pitch, yaw]
    {
        /*プローブの設置角度による補完*/
        // double buffer;
        // buffer = bryant_angle(0);
        // bryant_angle(0) = bryant_angle(1);
        // bryant_angle(1) = buffer;
        Eigen::VectorXd zxy_fixed_angle = Eigen::VectorXd::Constant(3,0);
        float roll = asin(sin(bryant_angle(1)) * cos(bryant_angle(2)) - cos(bryant_angle(1)) * sin(bryant_angle(0)) * sin(bryant_angle(2)));
        float pitch = atan2(cos(bryant_angle(1)) * sin(bryant_angle(0)) * cos(bryant_angle(2)) + sin(bryant_angle(1)) * sin(bryant_angle(2)) , cos(bryant_angle(1)) * cos(bryant_angle(0)));
        float sigma = atan2(cos(bryant_angle(0)) * sin(bryant_angle(2)), sin(bryant_angle(1)) * sin(bryant_angle(0)) * sin(bryant_angle(2)) + cos(bryant_angle(1)) * cos(bryant_angle(2)));
        zxy_fixed_angle << roll * 180/M_PI, pitch * 180/M_PI, sigma * 180/M_PI;

        return zxy_fixed_angle;
    }

    // Eigen::VectorXf ComputeKinematics::transformAngleExpression(const Eigen::VectorXf& bryant_angle) //bryant_angle = [roll, pitch, yaw]
    // {
    //     Eigen::VectorXf zxy_fixed_angle = Eigen::VectorXf::Constant(3,0);

    //     float sigma = atan2(sin(bryant_angle(2)) * cos(bryant_angle(0)) + cos(bryant_angle(2)) * sin(bryant_angle(1)) * sin(bryant_angle(0)),\
    //      cos(bryant_angle(2)) * cos(bryant_angle(1)));
    //     float pitch = atan2(sin(bryant_angle(2)) * sin(bryant_angle(0)) + sin(bryant_angle(2)) * sin(bryant_angle(1)) * cos(bryant_angle(0)),\
    //      cos(bryant_angle(1)) * cos(bryant_angle(0)));
    //     float roll = atan2(-(sin(bryant_angle(2)) * sin(bryant_angle(0)) - cos(bryant_angle(2)) * sin(bryant_angle(1)) * cos(bryant_angle(0))),\
    //      sqrt(pow(sin(bryant_angle(2)) * cos(bryant_angle(0)) + cos(bryant_angle(2)) * sin(bryant_angle(1)) * sin(bryant_angle(0)), 2) +\
    //      pow(cos(bryant_angle(2)) * cos(bryant_angle(1)), 2)));
    //     zxy_fixed_angle << roll * 180/M_PI, pitch * 180/M_PI, sigma * 180/M_PI;

    //     return zxy_fixed_angle;
    // }

};