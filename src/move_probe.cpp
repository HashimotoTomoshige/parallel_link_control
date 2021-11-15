/**********************************************************************\
 * [目的]: 
 * プローブ走査機構v2の順運動学と逆運動学を解くプログラム
 * [諸条件]:
 * 手先の姿勢表現はロール・ピッチ・ヨー角(ZYXオイラー角)
 * phi(z軸回り), theta(y軸回り), yaw(x軸回り) ※検査支持部と設定の仕方が異なるので要注意
 * 座標設定に関する詳細は「プローブ走査機構の制御.pdf」を参照
 * 
 * [コーディングに関して注意事項]
 * Eigenとstd::vectorでは四則演算は不可能
 * Eigen::VectorXdとEigen::MatrixXdは初期化しないと使えない
 * 
 * 
\**********************************************************************/

#include "parallel_link_control/move_probe.h"

namespace parallel_link_control
{
    MoveProbe::MoveProbe()
    {
        //write_flag_ = true;
    }

    Eigen::VectorXd MoveProbe::inputGoal()
    {
        Eigen::VectorXd r_goal = Eigen::VectorXd::Constant(6, 0);
        std::cout << "Input r_goal" << std::endl;
        std::cout << "x: ";
        std::cin >> r_goal(0);
        std::cout << "y: ";
        std::cin >> r_goal(1);
        std::cout << "z: ";
        std::cin >> r_goal(2);
        std::cout << "phi: ";
        std::cin >> r_goal(3);
        std::cout << "theta: ";
        std::cin >> r_goal(4);
        std::cout << "psi: ";
        std::cin >> r_goal(5);

        for(int i=3; i<6; i++) //度数法から弧度法に変換
        {
            r_goal(i) = r_goal(i) * M_PI/180;
        }
        return r_goal;
    }

    Eigen::VectorXd MoveProbe::threeRuns(const double& yaw)
    {
        Eigen::VectorXd r_goal = Eigen::VectorXd::Zero(6);
        double position_y = (linearmotor_encoder_value_(0) - encoder_initial_value_.at(0)) / complementary_value_;
        double position_z = -(linearmotor_encoder_value_(1) - encoder_initial_value_.at(1)) / complementary_value_;
        //std::cout<< "position_y: " << position_y << "   position_z: " << position_z << std::endl;
        //std::cout << "path_pattern: " << path_pattern_ << std::endl;
        if(abs(position_y) < 1  && abs(position_z) < 1 && write_flag_ == false && path_pattern_ ==0)
        {
            std::cout <<"OK1" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;
            r_goal << 0, -20, 0, 0, 0, yaw * M_PI/180;

            write_flag_ = true;
            path_pattern_ +=1;
        }
        if(abs(position_y - (-20)) < 1 && abs(position_z) < 1 && write_flag_ == false && path_pattern_==1)
        {
            r_goal << 0, -20, 50, 0, 0, yaw * M_PI/180;
            std::cout <<"OK2" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;

            write_flag_ = true;
            path_pattern_ +=1;
        }
        if(abs(position_y - (-20)) < 1 && abs(position_z - 50) < 1 && write_flag_ == false && path_pattern_==2)
        {
            r_goal << 0, -40, 50, 0, 0, yaw * M_PI/180;
            std::cout <<"OK3" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;

            write_flag_ = true;
            path_pattern_ +=1;
            
        }
        if(abs(position_y - (-40)) < 1 && abs(position_z - 50) < 1 && write_flag_ == false && path_pattern_==3)
        {
            r_goal << 0, -40, 0, 0, 0, yaw * M_PI/180;
            std::cout <<"OK4" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;

            write_flag_ = true;
            path_pattern_ +=1;;
        }
        if(abs(position_y - (-40)) < 1 && abs(position_z) < 1 && write_flag_ == false && path_pattern_==4)
        {
            r_goal << 0, 0, 0, 0, 0, yaw * M_PI/180;// r_goal << 0, -60, 0, phi, 0, yaw * M_PI/180;
            std::cout <<"OK5" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;

            write_flag_ = true;
            path_pattern_ +=1;
            start_running_ = false;
            path_pattern_ +=10;
        }
        if(abs(position_y - (-60)) < 1 && abs(position_z) < 1 && write_flag_ == false && path_pattern_==5)
        {
            r_goal << 0, -60, 50, 0, 0, yaw * M_PI/180;
            std::cout <<"OK6" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;

            write_flag_ = true;
            path_pattern_ +=1;
        }
        if((position_y - (-60)) < 1 && abs(position_z - 50) < 1 && write_flag_ == false && path_pattern_==6)
        {
            r_goal << 0, 0, 0, 0, 0, yaw * M_PI/180;
            std::cout <<"OK7" << std::endl;
            std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;

            write_flag_ = true;
            start_running_ = false;
            path_pattern_ +=1;
        }
        return r_goal;
    }

    Eigen::VectorXd MoveProbe::changeYaw(const double& deg)
    {
        Eigen::VectorXd r_goal = Eigen::VectorXd::Zero(6);
        r_goal << 0, 0, 0, 0, 0, deg * M_PI/180;
        return r_goal;
    }

    Eigen::VectorXd MoveProbe::initializePosition()
    {
        Eigen::VectorXd r_goal = Eigen::VectorXd::Zero(6);
        double position_y = (linearmotor_encoder_value_(0) - encoder_initial_value_.at(0)) / complementary_value_;
        double position_z = -(linearmotor_encoder_value_(1) - encoder_initial_value_.at(1)) / complementary_value_;
        std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;
        r_goal << 0, -position_y, -position_z, 0, 0, 0;
        return r_goal;
    }

    Eigen::VectorXd MoveProbe::getLinearMotorPosition()
    {   
        Eigen::VectorXd plane_position = Eigen::VectorXd::Zero(2);
        double position_y = (linearmotor_encoder_value_(0) - encoder_initial_value_.at(0)) / complementary_value_;
        double position_z = -(linearmotor_encoder_value_(1) - encoder_initial_value_.at(1)) / complementary_value_;
        std::cout <<"position_y: " << position_y << "position_z: " << position_z << std::endl;
        plane_position(0) = position_y;
        plane_position(1) = position_z;
        return plane_position;
    }

};
