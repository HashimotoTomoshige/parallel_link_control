#ifndef MOVE_PROBE_H
#define MOVE_PROBE_H

#include "include_files.h"

namespace parallel_link_control
{
    class MoveProbe
    {
    protected: //オブジェクトの属性を定義するもののみ記述
        Eigen::VectorXd linearmotor_encoder_value_ = Eigen::VectorXd::Zero(2);
        const std::vector<int> encoder_initial_value_ = {20303, 479757}; //直動モーター初期値におけるエンコーダーの値(モーターY, モーターZ)
        const double complementary_value_ = 100; //エンコーダの値からモーターの変位に変換
        int path_pattern_ = 0; //経路パターンを指定(同じ経路を歩まないようにする目的)

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Eigenをメンバ関数で用いる場合にこのマクロを追加
        MoveProbe(); //基底クラスのコンストラクタを実行するための関数
        Eigen::VectorXd inputGoal(); //目標姿勢を代入
        Eigen::VectorXd automaticRun(); //自動でプローブを走行
        Eigen::VectorXd automaticRun2(const double& deg); //プローブをyaw方向にdeg[°]回転させてから自動でプローブを走行
        Eigen::VectorXd changeYaw(const double& deg); //プローブをyaw方向にdeg[°]回転
        Eigen::VectorXd initializePosition();
        Eigen::VectorXd threeRuns(const double& yaw); //僧帽弁検出のための自動走行
        Eigen::VectorXd getLinearMotorPosition(); //直動モーターのエンコーダーの値を座標[mm]に変換
        bool write_flag_ = false; //自動走行でモーターに書き込むか否かの
        bool start_running_ = false;

    };
};

#endif

