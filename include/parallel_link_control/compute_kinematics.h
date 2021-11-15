#ifndef COMPUTE_KINEMATICS_H
#define COMPUTE_KINEMATICS_H

#include "include_files.h"
#include "parallel_link_control/move_probe.h"

namespace parallel_link_control
{
    class ComputeKinematics
    {
    protected: //オブジェクトの属性を定義するもののみ記述
        Eigen::VectorXd link_param_ = Eigen::VectorXd::Zero(6); //モーターの変位に依存するリンクパラメーター
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Eigenをメンバ関数で用いる場合にこのマクロを追加
        ComputeKinematics(); //基底クラスのコンストラクタを実行するための関数
        Eigen::VectorXd computePosture(Eigen::VectorXd r_goal); //目標の手先位置姿勢ベクトルから関節変位ベクトルを算出する関数
        Eigen::VectorXd transformAngleExpression(Eigen::VectorXd& bryant_angle); //
        
        Eigen::MatrixXd servo_velocity_ = Eigen::MatrixXd::Zero(3, 40);
        Eigen::MatrixXd zy_velocity_ = Eigen::VectorXd::Zero(2);
    };
};

#endif

