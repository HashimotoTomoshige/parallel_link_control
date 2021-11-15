/******************************************************************************
 * 僧帽弁探索のプログラム(auto_search.py)との組み合わせに際してこのノードの役割
 * 1. 走行をスタートする際にstep=1(int32)をpublish
 * 2. 走行が終了したら"probe_control/runs"をtrueでpublish
 * 2. "/probe_control/runs"でfalseを受け取ったらstep1に戻って走行やり直し
 * 3. "/probe_control/confirmed_position"で推論地最大の[x,y]配列を受け取ったらその位置に移動
 ******************************************************************************/

#include "parallel_link_control/compute_kinematics.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <unistd.h>
#include <string>

class ParallelLinkControl : public parallel_link_control::ComputeKinematics, public parallel_link_control::MoveProbe
{
protected:
    ros::NodeHandle nh_;
    ros::Publisher probe_control_pub_;
    ros::Publisher running_state_pub_;
    // ros::Publisher stepping_motor_pub_;
    ros::Publisher parasternal_approach_step_pub_;
    ros::Publisher finished_running_pub_;
    ros::Publisher initialize_position_pub_;
    ros::Publisher moved_to_confirmed_position_pub_;
    ros::Publisher moved_to_x_pub_;
    ros::Publisher move_yaw_pub_;
    ros::Publisher moved_to_confirmed_yaw_pub_;
    ros::Publisher record_servo_position_pub_;
    ros::Publisher move_pitch_pub_;
    
    ros::Publisher linear_pose_and_velocity_pub_;
    ros::Publisher parallel_pose_pub_;
    ros::Publisher linear_velocity_pub_;
    ros::Publisher parallel_reset_pub_;

    /*仮*/
    ros::Publisher start_step3_pub_;

    ros::Subscriber gui_sub_; //GUIからの命令を受け取る
    ros::Subscriber linear_motor_sub_;
    ros::Subscriber restart_running_sub_;
    ros::Subscriber confirmed_position_sub_;
    ros::Subscriber move_to_center_sub_;
    ros::Subscriber start_step5_sub_;
    ros::Subscriber turn_yaw_sub_;
    ros::Subscriber confirmed_yaw_sub_;
    ros::Subscriber servo_position_sub_;
    ros::Subscriber confirmed_pitch_sub_;
    ros::Subscriber joy_probe_control_sub_;
    ros::Subscriber joy_posture_initialization_sub_;
    ros::Subscriber parallel_flag_sub_;

    geometry_msgs::Twist probe_control_;
    std_msgs::Bool running_state_;
    // std_msgs::Float32 stepping_motor_angle_;
    std_msgs::Int32 parasternal_approach_step_;
    std_msgs::Bool finished_running_;
    std_msgs::Bool initialize_position_;
    std_msgs::Bool moved_to_confirmed_position_;
    std_msgs::Bool moved_to_x_;
    std_msgs::Bool moved_to_confirmed_yaw_;
    std_msgs::Bool record_servo_position_;
    std_msgs::Int16 move_yaw_;
    std_msgs::Bool move_pitch_;
    geometry_msgs::Point pitch_trajectory_planning_;
    std_msgs::Float32MultiArray linear_pose_and_velocity_;
    std_msgs::Float32MultiArray parallel_pose_;
    geometry_msgs::Point linear_velocity_;
    std_msgs::Float32 parallel_reset_;
    /*仮*/
    std_msgs::Bool step3_;

    bool joy_posture_flag_;

public:
    ParallelLinkControl();
    void publishProbeControl(const Eigen::VectorXd& q);
    void publishRunningState();
    void publishSteppingMotorAngle(const double& q5);
    void publishParasternalApproachStep(const int32_t& step_number);
    void publishFinishedRunning();
    void publishInitialization();
    void publishMovedToConfirmedPosition();
    void publishMovedToX();
    void publishMoveYaw(const int16_t& direction);
    void publishMovedToConfirmedYaw();

    void publishMovePitch(const bool& pitch_state);
    void publishLinearVelocity(const Eigen::VectorXd& r);

    void publishLinearPoseAndVelocity(const Eigen::VectorXd& r);
    void publishParallelPose(const Eigen::VectorXd& r, const float& traj_time);
    void publishParallelReset();

    /*仮*/
    void publishStartStep3();

    void linearmotorCallback(const std_msgs::Int32MultiArray& msg);
    void guiCallback(const std_msgs::Bool& msg);
    void runsCallback(const std_msgs::Bool& msg);
    void confirmedpositionCallback(const std_msgs::Float32MultiArray& msg);
    void movetocenterCallback(const std_msgs::Bool& msg);
    void startstep5Callback(const std_msgs::Bool& msg);
    void turnyawCallback(const std_msgs::Bool& msg);
    void confirmedyawCallback(const std_msgs::Float32& msg);
    void confirmedpitchCallback(const std_msgs::Float32& msg);
    void parallelFlagCallback(const std_msgs::Bool& msg);

    void joyPostureInitializationCallback(const std_msgs::Bool& msg);
    void joyProbeControlCallback(const geometry_msgs::Twist& msg);

    int32_t step_ = 0;
    bool yaw_change_ = false;
    Eigen::VectorXd r_goal_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd bryant_angle_ = Eigen::VectorXd::Zero(3);
    bool initial_flag_ = true;
    bool parallel_flag_;
    bool moved_to_x_flag_;

    std::string mode_ = "";
};

ParallelLinkControl::ParallelLinkControl() : parallel_link_control::ComputeKinematics(), parallel_link_control::MoveProbe()
{
    probe_control_pub_ = nh_.advertise<geometry_msgs::Twist>("probe_control", 1);
    running_state_pub_ = nh_.advertise<std_msgs::Bool>("start_running", 1);
    // stepping_motor_pub_ = nh_.advertise<std_msgs::Float32>("ref_motor_angle", 1);
    parasternal_approach_step_pub_ = nh_.advertise<std_msgs::Int32>("step", 1);
    finished_running_pub_ = nh_.advertise<std_msgs::Bool>("three_runs", 1);
    initialize_position_pub_ = nh_.advertise<std_msgs::Bool>("initialization", 1);
    moved_to_confirmed_position_pub_ = nh_.advertise<std_msgs::Bool>("moved_to_confirmed_xy", 1);
    moved_to_x_pub_ = nh_.advertise<std_msgs::Bool>("moved_to_x", 1);
    move_yaw_pub_ = nh_.advertise<std_msgs::Int16>("move_yaw", 1);
    moved_to_confirmed_yaw_pub_ = nh_.advertise<std_msgs::Bool>("moved_to_confirmed_yaw", 1);
    record_servo_position_pub_ = nh_.advertise<std_msgs::Bool>("record_servo_position", 1);
    move_pitch_pub_ = nh_.advertise<std_msgs::Bool>("move_pitch", 1);

    linear_pose_and_velocity_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("linear_pose_and_velocity", 1);
    linear_velocity_pub_ = nh_.advertise<geometry_msgs::Point>("linear_velocity", 1);
    parallel_pose_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("ref_motor_angle", 1);
    parallel_reset_pub_ = nh_.advertise<std_msgs::Float32>("reset", 1);
    /*仮*/
    start_step3_pub_ = nh_.advertise<std_msgs::Bool>("start_step", 1);

    linear_motor_sub_ = nh_.subscribe("current_pos", 1, &ParallelLinkControl::linearmotorCallback, this); //thisポインタ: メッセージを受け取ったときに呼び出す関数がクラスの中にある場合にクラスの実体を指定
    gui_sub_ = nh_.subscribe("start_running", 1, &ParallelLinkControl::guiCallback, this);
    restart_running_sub_ = nh_.subscribe("rerun", 1, &ParallelLinkControl::runsCallback, this);
    confirmed_position_sub_ = nh_.subscribe("confirmed_xy", 1, &ParallelLinkControl::confirmedpositionCallback, this);
    move_to_center_sub_ = nh_.subscribe("move_to_center", 1, &ParallelLinkControl::movetocenterCallback, this);
    start_step5_sub_ = nh_.subscribe("start_step5", 1, &ParallelLinkControl::startstep5Callback, this);
    turn_yaw_sub_ = nh_.subscribe("turn_yaw", 1, &ParallelLinkControl::turnyawCallback, this);
    confirmed_yaw_sub_ = nh_.subscribe("confirmed_yaw", 1, &ParallelLinkControl::confirmedyawCallback, this);
    confirmed_pitch_sub_ = nh_.subscribe("confirmed_pitch", 1, &ParallelLinkControl::confirmedpitchCallback, this);
    joy_probe_control_sub_ = nh_.subscribe("joy_probe_control", 1, &ParallelLinkControl::joyProbeControlCallback, this);
    joy_posture_initialization_sub_ = nh_.subscribe("joy_posture_initialization", 1, &ParallelLinkControl::joyPostureInitializationCallback, this);
    parallel_flag_sub_ = nh_.subscribe("parallel_flag", 1, &ParallelLinkControl::parallelFlagCallback, this);

    joy_posture_flag_ = false;
    parallel_flag_ = true;
    moved_to_x_flag_ = false;
}

void ParallelLinkControl::publishProbeControl(const Eigen::VectorXd& r_p)
{
    probe_control_.linear.x = r_p(0);
    probe_control_.linear.y = r_p(1); //マイナスをつけているのはモーターの取り付け向きが想定と逆になったため
    probe_control_.linear.z = -r_p(2);
    probe_control_.angular.x = r_p(3);
    probe_control_.angular.y = r_p(4);
    probe_control_.angular.z = r_p(5);
    probe_control_pub_.publish(probe_control_);
    std::cout << "three runs path_pattern: " << path_pattern_ << std::endl;

    write_flag_ = false;
    Eigen::VectorXd bryant_angle = Eigen::VectorXd::Zero(3);
    bryant_angle << r_p(3), r_p(4), r_p(5);
    Eigen::VectorXd zxy_fixed_angle = Eigen::VectorXd::Zero(3);
    
    zxy_fixed_angle = transformAngleExpression(bryant_angle);
    for(int i=0; i<3; i++)
    {
        r_goal_(i+3) = zxy_fixed_angle(i);
    }
    // r_goal(3) = zxy_fixed_angle(0);
    // r_goal(4) = zxy_fixed_angle(1);
    // r_goal(5) = zxy_fixed_angle(2);
    std::cout << r_goal_ << std::endl;
    
    publishParallelPose(r_goal_, 6);
}

void ParallelLinkControl::publishRunningState()
{
    running_state_.data = start_running_;
    running_state_pub_.publish(running_state_);
}

void ParallelLinkControl::publishParasternalApproachStep(const int32_t& step_number)
{
    parasternal_approach_step_.data = step_number;
    step_ = parasternal_approach_step_.data;
    parasternal_approach_step_pub_.publish(parasternal_approach_step_);
}

void ParallelLinkControl::publishFinishedRunning()
{
    finished_running_.data = true;
    finished_running_pub_.publish(finished_running_);
}

void ParallelLinkControl::publishInitialization()
{
    initialize_position_.data = true;
    initialize_position_pub_.publish(initialize_position_);
    sleep(1);

}

void ParallelLinkControl::publishMovedToConfirmedPosition()
{
    moved_to_confirmed_position_.data = true;
    moved_to_confirmed_position_pub_.publish(moved_to_confirmed_position_);
}

void ParallelLinkControl::publishMovedToX()
{
    moved_to_x_.data = true;
    moved_to_x_pub_.publish(moved_to_x_);
    std::cout <<"published moved to x" << std::endl;
    moved_to_x_flag_ = false;
}

void ParallelLinkControl::publishMoveYaw(const int16_t& direction)
{
    
    r_goal_(3) = 0;
    r_goal_(4) = 0;
    if(direction == 0)
    {
        std::cout << "move yaw finish" << std::endl;
    }
    else if(direction < 0)
    {
        r_goal_(5) = -M_PI;
        publishParallelPose(computePosture(r_goal_), 6);
    }
    else if(direction > 0)
    {
        r_goal_(5) = M_PI;
        publishParallelPose(computePosture(r_goal_), 12);
    }
    move_yaw_.data = direction;
    move_yaw_pub_.publish(move_yaw_);
    std::cout << "move yaw is published" << std::endl;
}

void ParallelLinkControl::publishMovedToConfirmedYaw()
{

    moved_to_confirmed_yaw_.data = true;
    moved_to_confirmed_yaw_pub_.publish(moved_to_confirmed_yaw_);
}

void ParallelLinkControl::publishLinearPoseAndVelocity(const Eigen::VectorXd& r)
{
    linear_pose_and_velocity_.data.resize(4);
    linear_pose_and_velocity_.data[0] = r(2);
    linear_pose_and_velocity_.data[1] = r(1);
    linear_pose_and_velocity_.data[2] = ((r(2)>0) - (r(2)<0)) * 5;
    linear_pose_and_velocity_.data[3] = ((r(1)>0) - (r(1)<0)) * 5;
    linear_pose_and_velocity_pub_.publish(linear_pose_and_velocity_);
}

void ParallelLinkControl::publishLinearVelocity(const Eigen::VectorXd& r)
{
    linear_velocity_.y = r(1) / 10;
    linear_velocity_.z = -r(2) / 10;
    linear_velocity_pub_.publish(linear_velocity_);

}

void ParallelLinkControl::publishParallelPose(const Eigen::VectorXd& r, const float& traj_time)
{
    // float r_3 = std::floor(r(3) * 100) * 0.01;
    // float r_4 = std::floor(r(4) * 100) * 0.01;
    // float r_5 = std::floor(r(5) * 100) * 0.01;
    std::cout << "publish parallel pose" << std::endl;
    parallel_pose_.data.resize(4);
    parallel_pose_.data[0] = r(3);
    parallel_pose_.data[1] = r(4);
    parallel_pose_.data[2] = r(5);
    parallel_pose_.data[3] = traj_time;
    parallel_pose_pub_.publish(parallel_pose_);
}

void ParallelLinkControl::publishMovePitch(const bool& pitch_state)
{
    move_pitch_.data = pitch_state;
    move_pitch_pub_.publish(move_pitch_);
    std::cout << "Publish move_pitch: " << pitch_state << std::endl;
}

void ParallelLinkControl::publishStartStep3()
{
    step3_.data = true;
    start_step3_pub_.publish(step3_);
}

void ParallelLinkControl::publishParallelReset()
{
    parallel_reset_.data = 1.0;
    parallel_reset_pub_.publish(parallel_reset_);
}

/*直動モーターのエンコーダーの値をsubscribe*/
void ParallelLinkControl::linearmotorCallback(const std_msgs::Int32MultiArray& msg)
{
    for(int i=0; i<2; i++)
    {
        linearmotor_encoder_value_(i) =msg.data[i];
        // ROS_INFO("Subscribe linearmotor_encoder_value[%d] is %f", i, linearmotor_encoder_value_(i));
    }
}

void ParallelLinkControl::guiCallback(const std_msgs::Bool& msg)
{
    if(msg.data == true/* && initial_flag_ == true*/)
    {
        start_running_ = msg.data;
        publishParasternalApproachStep(1);
        ROS_INFO("Subscribe start_running");
        // sleep(11);
        initial_flag_ = false;
        // sleep(5);
    }
    else
    {
        std::cout << "ok1" << std::endl;
        Eigen::VectorXd r_p = computePosture(initializePosition());
        std::cout << "ok1" << std::endl;
        // std::cout << linearmotor_encoder_value_(0) << ",  " << linearmotor_encoder_value_(2) << std::endl;
        publishInitialization();
        std::cout << r_p;
        publishProbeControl(r_p);
    }
}

void ParallelLinkControl::runsCallback(const std_msgs::Bool& msg)
{
    if(msg.data == true)
    {
        path_pattern_ = 0;
        write_flag_ = true;
        sleep(5); //プローブが原点に移動するまでの秒数
        Eigen::VectorXd r_p = computePosture(changeYaw(45)); //プローブをYaw方向に-90°回転
        publishProbeControl(r_p);
        sleep(3);
        yaw_change_ = true;
        start_running_ = true;
        r_goal_(5) = 45 *M_PI/180;
        publishRunningState();
    }
}

void ParallelLinkControl::confirmedpositionCallback(const std_msgs::Float32MultiArray& msg)
{
    if(mode_ == "Shiotani")
    {
        step_ = 2;
        r_goal_(1) = -35.0; //僧帽弁位置水平右方向
        r_goal_(2) = 5.0; //僧帽弁位置鉛直上方向
        write_flag_ = true;
    }
    else if(mode_ == "Kumagai")
    {
        step_ = 2;
        r_goal_(1) = -20.0; //僧帽弁位置水平右方向
        r_goal_(2) = 13.0; //僧帽弁位置鉛直上方向
        write_flag_ = true;
    }
    else if(step_ == 1)
    {
        step_ = 2;
        r_goal_(1) = msg.data[0];
        r_goal_(2) = msg.data[1];
        write_flag_ = true;
    }
    else if(step_ == 3)
    {
        std::cout << "confirmed position sec time" << std::endl;
        step_ = 2;
        r_goal_(1) = msg.data[0];
        r_goal_(2) = msg.data[1];
        write_flag_ = true;
    }
}

void ParallelLinkControl::movetocenterCallback(const std_msgs::Bool& msg)
{
    step_ = 4;
    if(msg.data == true)
    {
        r_goal_(1) += 2; //画像x方向に+5[mm]
        std::cout << "x + 2" << std::endl;
    }
    else if(msg.data == false)
    {
        r_goal_(1) -= 2; //画像x方向に-5[mm]
        std::cout << "x - 2" << std::endl;
    }
    write_flag_ = true;
    moved_to_x_flag_ = true;
}

void ParallelLinkControl::startstep5Callback(const std_msgs::Bool& msg)
{
    if(msg.data == true)
    {
        step_ = 5;
        // sleep(1);
        std::cout << "-------------move yaw-------------" << std::endl;
        publishMoveYaw(-1);
        sleep(5);

    }
}

void ParallelLinkControl::turnyawCallback(const std_msgs::Bool& msg)
{
    // if (msg.data == true && step_ == 5)
    // {
    //     publishMoveYaw(0);
    //     sleep(2);
    //     publishMoveYaw(1);
    // }   
}

void ParallelLinkControl::confirmedyawCallback(const std_msgs::Float32& msg)
{
    if(mode_ == "Shiotani")
    {
        // publishMoveYaw(0);
        // sleep(5);
        r_goal_(5) = 40 * M_PI/180;
        publishParallelPose(computePosture(r_goal_), 3);
        step_ = 6;
    }
    if(mode_ == "Kumagai")
    {
        // publishMoveYaw(0);
        // sleep(13);
        r_goal_(5) = 45 * M_PI/180;
        publishParallelPose(computePosture(r_goal_), 3);
        step_ = 6;
    }
    if(step_ == 5)
    {
        // publishMoveYaw(0);
        sleep(2);
        r_goal_(5) = msg.data;
        publishParallelPose(computePosture(r_goal_), 3);
        step_ = 6;
    }
}

bool pitch_flag = false;
bool yaw_flag = false;
void ParallelLinkControl::parallelFlagCallback(const std_msgs::Bool& msg)
{
    std::cout << "subscribe_parallel_flag" << std::endl;
    parallel_flag_ = true;
    if(msg.data == true && step_ == 5 && yaw_flag == true)
    {
        publishMoveYaw(0);
        std::cout << "Yaw finished" << std::endl;
    }
    else if(msg.data == true && step_ == 5 && yaw_flag == false)
    {
        publishMoveYaw(1);
        yaw_flag = true;
    }
    if(msg.data == true && step_ == 6)
    {
        if(pitch_flag == true)
        {
            r_goal_(4) = 20 * M_PI/180;
            step_ = 7;
            std::cout << "pitch_flag(20): " << pitch_flag << std::endl;
        }
        else if(pitch_flag == false)
        {
            publishMovedToConfirmedYaw();
            r_goal_(4) = -20 * M_PI/180;
            pitch_flag = true;
            std::cout << "pitch_flag(-20): " << pitch_flag << std::endl;
            publishMovePitch(1);
        }
        publishParallelPose(computePosture(r_goal_), 3.5);
    }
    else if(msg.data == true && step_ == 7)
    {
        publishMovePitch(0);
    }
}

void ParallelLinkControl::confirmedpitchCallback(const std_msgs::Float32& msg) //確定されたピッチ姿勢に移動するプログラム
{
    if(mode_ == "Shiotani")
    {
        sleep(1);
        r_goal_(4) = 0 * M_PI/180;
        publishParallelPose(computePosture(r_goal_), 2);
        std::cout << "confirmed pitch published" << std::endl;
    }
    if(mode_ == "Kumagai")
    {
        sleep(1);
        r_goal_(4) = 0 * M_PI/180;
        publishParallelPose(computePosture(r_goal_), 2);
        std::cout << "confirmed pitch published" << std::endl;
    }
    else
    {
        sleep(1);
        r_goal_(4) = msg.data;
        publishParallelPose(computePosture(r_goal_), 2);
        std::cout << "confirmed pitch published" << std::endl;
    }
}



void ParallelLinkControl::joyPostureInitializationCallback(const std_msgs::Bool& msg) //プローブの姿勢のみ初期化, 位置はそのまま
{
    if(msg.data == true)
    {
        r_goal_(3) = 0;
        r_goal_(4) = 0;
        r_goal_(5) = 0;
        publishProbeControl(computePosture(r_goal_));
        std::cout << "joy posture is initialized." << std::endl;
    }
}

/*10/29*/
void ParallelLinkControl::joyProbeControlCallback(const geometry_msgs::Twist& msg)
{
    if(msg.angular.x == 0 && msg.angular.y == 0 && msg.angular.z == 0) //プローブ位置調整モード
    {
        if(true) //プローブ位置調整モード
        {
            std::cout << "[joy] probe position control mode" << std::endl;
            if(msg.linear.y > 0.5)
            {
                r_goal_(1) = 50;
            }
            else if(msg.linear.y < -0.5)
            {
                r_goal_(1) = -50;
            }
            else
            {
                r_goal_(1) = 0;
            }
            if(msg.linear.z > 0.5)
            {
                r_goal_(2) = 50;
            }
            else if(msg.linear.z < -0.5)
            {
                r_goal_(2) = -50;
            }
            else
            {
                r_goal_(2) = 0;
            }
            publishLinearVelocity(computePosture(r_goal_));

        }
        else //プローブ姿勢停止
        {
            /* code */
        }
        
    }
    else if(msg.angular.x != 0 || msg.angular.y != 0 || msg.angular.z != 0)
    {
        joy_posture_flag_ = true;
        std::cout << "[joy] probe posture control mode" << std::endl;
        
        float traj_time = 0;
        if(msg.angular.x >= 0.2 && parallel_flag_ == true)// if(msg.angular.x >= 0.2 && msg.angular.x < 0.5)
        {
            parallel_flag_ = false;
            bryant_angle_(2) += 5 * M_PI/180;
            traj_time = 1;
        }
        else if(msg.angular.x <= -0.2 && parallel_flag_ == true)// else if(msg.angular.x > -0.5 && msg.angular.x <= -0.2)
        {
            parallel_flag_ = false;
            bryant_angle_(2) -= 5 * M_PI/180;
            traj_time = 1;
        }
        // else if(msg.angular.x >= 0.5 && msg.angular.x < 0.8)
        // {
        //     bryant_angle_(2) += 5 * M_PI/180;
        //     traj_time = 1.5;
        // }
        // else if(msg.angular.x > -0.8 && msg.angular.x <= -0.2)
        // {
        //     bryant_angle_(2) -= 5 * M_PI/180;
        //     traj_time = 1.5;
        // }
        // else if(msg.angular.x >= 0.8)
        // {
        //     bryant_angle_(2) += 10 * M_PI/180;
        //     traj_time = 2;
        // }
        // else if(msg.angular.x <= -0.8)
        // {
        //     bryant_angle_(2) -= 10 * M_PI/180;
        //     traj_time = 2;
        // }
        else
        {
            std::cout << "sigma stop" << std::endl;
        }

        if(msg.angular.y >= 0.2 && parallel_flag_ == true)// if(msg.angular.y >= 0.2 && msg.angular.y < 0.5)
        {
            parallel_flag_ = false;
            bryant_angle_(1) -= 5 * M_PI/180;
            traj_time = 1;
        }
        else if(msg.angular.y <= -0.2 && parallel_flag_ == true)// else if(msg.angular.y > -0.5 && msg.angular.y <= -0.2)
        {
            parallel_flag_ = false;
            bryant_angle_(1) += 5 * M_PI/180;
            traj_time = 1;
        }
        // else if(msg.angular.y >= 0.5 && msg.angular.y < 0.8)
        // {
        //     bryant_angle_(1) += 5 * M_PI/180;
        //     traj_time = 1.5;
        // }
        // else if(msg.angular.y > -0.8 && msg.angular.y <= -0.2)
        // {
        //     bryant_angle_(1) -= 5 * M_PI/180;
        //     traj_time = 1.5;
        // }
        // else if(msg.angular.y >= 0.8)
        // {
        //     bryant_angle_(1) += 10 * M_PI/180;
        //     traj_time = 2;
        // }
        // else if(msg.angular.y <= -0.8)
        // {
        //     bryant_angle_(1) -= 10 * M_PI/180;
        //     traj_time = 2;
        // }
        else
        {
            std::cout << "pitch stopped" << std::endl;
        }

        if(msg.angular.z >= 0.2 && parallel_flag_ == true)// if(msg.angular.z >= 0.2 && msg.angular.z < 0.5)
        {
            parallel_flag_ = false;
            bryant_angle_(0) += 5 * M_PI/180;
            traj_time = 1;
        }
        else if(msg.angular.z <= -0.2 && parallel_flag_ == true)// else if(msg.angular.z > -0.5 && msg.angular.z <= -0.2)
        {
            parallel_flag_ = false;
            bryant_angle_(0) -= 5 * M_PI/180;
            traj_time = 1;
        }
        // else if(msg.angular.z >= 0.5 && msg.angular.z < 0.8)
        // {
        //     bryant_angle_(0) += 5 * M_PI/180;
        //     traj_time = 1.5;
        // }
        // else if(msg.angular.z > -0.8 && msg.angular.z <= -0.2)
        // {
        //     bryant_angle_(0) -= 5 * M_PI/180;
        //     traj_time = 1.5;
        // }
        // else if(msg.angular.z >= 0.8)
        // {
        //     bryant_angle_(0) += 10 * M_PI/180;
        //     traj_time = 2;
        // }
        // else if(msg.angular.z <= -0.8)
        // {
        //     bryant_angle_(0) -= 10 * M_PI/180;
        //     traj_time = 2;
        // }
        else
        {
            std::cout << "roll stop" << std::endl;
        }

        r_goal_(3) = bryant_angle_(0);
        r_goal_(4) = bryant_angle_(1);
        r_goal_(5) = bryant_angle_(2);


        publishParallelPose(computePosture(r_goal_), traj_time);
        // usleep((traj_time + 1)  * 1000000);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parallel_link_control_node");
    ParallelLinkControl probe1;
    ros::Rate rate(10); //ループの頻度を設定するためのオブジェクトを作成。この場合は10Hz、1秒間に10回数、1ループ100ms。
    Eigen::VectorXd r_p = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd r_goal = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd plane_position = Eigen::VectorXd::Zero(2);
    double tolerance = 0.5; //プローブ移動の際の許容誤差[mm]
    
    std::cout << "input mode" << std::endl;
    std::getline(std::cin, probe1.mode_);
    //probe1.start_running_ = true;
    bool finished_flag = false;
    if(probe1.mode_ == "joy" || probe1.mode_ == "parallel")
    {
        probe1.publishParallelReset();
    }

    // if(mode !="0")
    // {   
    //     std::cout << "Input 0(1st)" << std::endl;
    //     std::cin >> r_goal(4);
    //     q = probe1.computePosture(r_goal, "Euler");
    //     probe1.publishProbeControl(q);
    //     std::cout << "Input 0(2nd)" << std::endl;
    //      std::cin >> r_goal(4);
    //     q = probe1.computePosture(r_goal, "Euler");
    //     probe1.publishProbeControl(q);
    //     sleep(5);
    // }
    
    
    while (ros::ok()) // このノードが使える間は無限ループする
    {
        if(probe1.mode_ == "automatic") //僧帽弁探索モード
        {
            if(probe1.step_ == 1)
            {
                if(probe1.yaw_change_ == false)
                {
                    r_goal = probe1.threeRuns(0); // 8/17: 僧帽弁探索用に関数を変えるかも
                    sleep(0.5);
                    if(probe1.write_flag_)
                    {
                        if(probe1.start_running_ == true)
                        {
                            finished_flag = false;
                            std::cout << "r_goal: " << r_goal << std::endl;
                            probe1.r_goal_ = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(probe1.r_goal_);
                        }
                        else if(probe1.start_running_ == false && finished_flag == false)
                        {
                            probe1.publishRunningState();
                            probe1.publishFinishedRunning();
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                            finished_flag = true;
                        }
                    }
                }
                else if(probe1.yaw_change_ == true)
                {
                    r_goal = probe1.threeRuns(45); // 8/17: 僧帽弁探索用に関数を変えるかも
                    if(probe1.write_flag_)
                    {
                        if(probe1.start_running_ == true)
                        {
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                        }
                        else if(probe1.start_running_ == false)
                        {
                            probe1.publishRunningState();
                            probe1.publishFinishedRunning();
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                        }
                    }
                }
            }
            else if(probe1.step_ == 2)
            {
                // std::cout << "step2" << std::endl;
                plane_position = probe1.getLinearMotorPosition();
                if(probe1.write_flag_ == true)
                {
                    if(true)
                    {
                        std::cout << "abs(plane_position(0)): " << abs(plane_position(0)) << ", abs(plane_position(1)): " << abs(plane_position(1)) << std::endl;
                        sleep(7);
                        probe1.publishProbeControl(probe1.computePosture(probe1.r_goal_));
                        std::cout << "confirmed_position published" << std::endl;
                    }    
                }
                else if(abs(plane_position(0) - probe1.r_goal_(1)) < tolerance && abs(plane_position(1) - probe1.r_goal_(2)) < tolerance)
                {
                    std::cout << "get to comfirmed position" << std::endl;
                    probe1.publishMovedToConfirmedPosition();
                    probe1.step_ = 3;
                }
                else
                {
                    std::cout << "abs(plane_position(0)): " << abs(plane_position(0)) << ", abs(plane_position(1)): " << abs(plane_position(1)) << std::endl;
                    std::cout <<"r_goal_(1): " << probe1.r_goal_(1) << ", r_goal_(2): " << probe1.r_goal_(2) << std::endl;
                }
            }
            else if(probe1.step_ == 4)
            {
                plane_position = probe1.getLinearMotorPosition();
                if(probe1.write_flag_ == true)
                {
                    probe1.publishProbeControl(probe1.computePosture(probe1.r_goal_));
                }
                else if(abs(plane_position(0) - probe1.r_goal_(1)) < tolerance && abs(plane_position(1) - probe1.r_goal_(2)) < tolerance && probe1.moved_to_x_flag_)
                {
                    probe1.publishMovedToX();
                }
                
            }
            else if(probe1.step_ == 5)
            {
                // std::cout << "!!!!!!!!step 5!!!!!!!!!" << std::endl;
            }
        }
        else if(probe1.mode_ == "2") //手動走行モード
        {
            r_p = probe1.computePosture(probe1.inputGoal());
            std::cout << r_p << std::endl;
            probe1.publishProbeControl(r_p);
            probe1.getLinearMotorPosition();
        }
        else if(probe1.mode_ == "3")
        {
            
            probe1.publishStartStep3();
            probe1.publishMovedToConfirmedYaw();
            r_goal(4) = -20 * M_PI/180;
            r_p = probe1.computePosture(r_goal);
            std::cout << "published" << std::endl;
            probe1.publishProbeControl(r_p);
            sleep(5);
            probe1.mode_ = "";
        }
        else if(probe1.mode_ == "4") //ピッチ軌道制御モード
        {
            r_p = probe1.computePosture(probe1.inputGoal());
            std::cout << r_p << std::endl;
            probe1.publishProbeControl(r_p);
            // r_p = probe1.pitchTrajectoryPlanning(probe1.inputGoal());
            // probe1.publishProbeControl(q);
            sleep(3);
            probe1.getLinearMotorPosition();
        }
        else if(probe1.mode_ == "joy")
        {
        }
        else if(probe1.mode_ == "parallel")
        {
            float roll, pitch, yaw, z, y;
            std::cout << "input roll pitch yaw" << std::endl;
            std::cin >> roll >> pitch >> yaw;
            std::cout << "input y z" << std::endl;
            std::cin >> z >> y;
            Eigen::VectorXd bryant_angle = Eigen::VectorXd::Zero(3);
            Eigen::VectorXd zxy_fixed_angle = Eigen::VectorXd::Zero(3);
            bryant_angle << roll * M_PI/180, pitch * M_PI/180, yaw * M_PI/180;
            // r_goal(0) = 0;
            // r_goal(1) = y;
            // r_goal(2) = z;
            // r_goal(3) = bryant_angle(0);
            // r_goal(4) = bryant_angle(1);
            // r_goal(5) = bryant_angle(2);
            zxy_fixed_angle = probe1.transformAngleExpression(bryant_angle);
            std::cout << "roll: " << zxy_fixed_angle(0) << "pitch: " << zxy_fixed_angle(1) << "sigma: " << zxy_fixed_angle(2) << std::endl;
            for(int i=0; i<3; i++)
            {
            r_goal(i+3) = zxy_fixed_angle(i);
            }
            r_goal(3) = zxy_fixed_angle(0);
            r_goal(4) = zxy_fixed_angle(1);
            r_goal(5) = zxy_fixed_angle(2);
            std::cout << r_goal << std::endl;
            
            probe1.publishParallelPose(r_goal, 3);

        }
        if(probe1.mode_ == "Shiotani") //プレスリリース塩谷用モード
        {
            if(probe1.step_ == 1)
            {
                if(probe1.yaw_change_ == false)
                {
                    r_goal = probe1.threeRuns(0); // 8/17: 僧帽弁探索用に関数を変えるかも
                    sleep(0.5);
                    if(probe1.write_flag_)
                    {
                        if(probe1.start_running_ == true)
                        {
                            finished_flag = false;
                            std::cout << "r_goal: " << r_goal << std::endl;
                            probe1.r_goal_ = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(probe1.r_goal_);
                        }
                        else if(probe1.start_running_ == false && finished_flag == false)
                        {
                            probe1.publishRunningState();
                            probe1.publishFinishedRunning();
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                            finished_flag = true;
                        }
                    }
                }
                else if(probe1.yaw_change_ == true)
                {
                    r_goal = probe1.threeRuns(45); // 8/17: 僧帽弁探索用に関数を変えるかも
                    if(probe1.write_flag_)
                    {
                        if(probe1.start_running_ == true)
                        {
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                        }
                        else if(probe1.start_running_ == false)
                        {
                            probe1.publishRunningState();
                            probe1.publishFinishedRunning();
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                        }
                    }
                }
            }
            else if(probe1.step_ == 2)
            {
                // std::cout << "step2" << std::endl;
                plane_position = probe1.getLinearMotorPosition();
                if(probe1.write_flag_ == true)
                {
                    if(true)
                    {
                        std::cout << "abs(plane_position(0)): " << abs(plane_position(0)) << ", abs(plane_position(1)): " << abs(plane_position(1)) << std::endl;
                        sleep(7);
                        probe1.publishProbeControl(probe1.computePosture(probe1.r_goal_));
                        std::cout << "confirmed_position published" << std::endl;
                    }    
                }
                else if(abs(plane_position(0) - probe1.r_goal_(1)) < tolerance && abs(plane_position(1) - probe1.r_goal_(2)) < tolerance)
                {
                    std::cout << "get to comfirmed position" << std::endl;
                    probe1.publishMovedToConfirmedPosition();
                    probe1.step_ = 3;
                }
                else
                {
                    std::cout << "abs(plane_position(0)): " << abs(plane_position(0)) << ", abs(plane_position(1)): " << abs(plane_position(1)) << std::endl;
                    std::cout <<"r_goal_(1): " << probe1.r_goal_(1) << ", r_goal_(2): " << probe1.r_goal_(2) << std::endl;
                }
            }
            else if(probe1.step_ == 4)
            {
                plane_position = probe1.getLinearMotorPosition();
                if(probe1.write_flag_ == true)
                {
                    probe1.publishProbeControl(probe1.computePosture(probe1.r_goal_));
                }
                else if(abs(plane_position(0) - probe1.r_goal_(1)) < tolerance && abs(plane_position(1) - probe1.r_goal_(2)) < tolerance && probe1.moved_to_x_flag_)
                {
                    probe1.publishMovedToX();
                }
                
            }
            else if(probe1.step_ == 5)
            {
                // std::cout << "!!!!!!!!step 5!!!!!!!!!" << std::endl;
            }
        }
        if(probe1.mode_ == "Kumagai") //プレスリリース塩谷用モード
        {
            if(probe1.step_ == 1)
            {
                if(probe1.yaw_change_ == false)
                {
                    r_goal = probe1.threeRuns(0); // 8/17: 僧帽弁探索用に関数を変えるかも
                    sleep(0.5);
                    if(probe1.write_flag_)
                    {
                        if(probe1.start_running_ == true)
                        {
                            finished_flag = false;
                            std::cout << "r_goal: " << r_goal << std::endl;
                            probe1.r_goal_ = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(probe1.r_goal_);
                        }
                        else if(probe1.start_running_ == false && finished_flag == false)
                        {
                            probe1.publishRunningState();
                            probe1.publishFinishedRunning();
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                            finished_flag = true;
                        }
                    }
                }
                else if(probe1.yaw_change_ == true)
                {
                    r_goal = probe1.threeRuns(45); // 8/17: 僧帽弁探索用に関数を変えるかも
                    if(probe1.write_flag_)
                    {
                        if(probe1.start_running_ == true)
                        {
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                        }
                        else if(probe1.start_running_ == false)
                        {
                            probe1.publishRunningState();
                            probe1.publishFinishedRunning();
                            r_p = probe1.computePosture(r_goal);
                            probe1.publishProbeControl(r_p);
                        }
                    }
                }
            }
            else if(probe1.step_ == 2)
            {
                // std::cout << "step2" << std::endl;
                plane_position = probe1.getLinearMotorPosition();
                if(probe1.write_flag_ == true)
                {
                    if(true)
                    {
                        // std::cout << "abs(plane_position(0)): " << abs(plane_position(0)) << ", abs(plane_position(1)): " << abs(plane_position(1)) << std::endl;
                        sleep(7);
                        probe1.publishProbeControl(probe1.computePosture(probe1.r_goal_));
                        std::cout << "confirmed_position published" << std::endl;
                    }    
                }
                else if(abs(plane_position(0) - probe1.r_goal_(1)) < tolerance && abs(plane_position(1) - probe1.r_goal_(2)) < tolerance)
                {
                    std::cout << "get to comfirmed position" << std::endl;
                    probe1.publishMovedToConfirmedPosition();
                    probe1.step_ = 3;
                }
                else
                {
                    // std::cout << "abs(plane_position(0)): " << abs(plane_position(0)) << ", abs(plane_position(1)): " << abs(plane_position(1)) << std::endl;
                    std::cout <<"r_goal_(1): " << probe1.r_goal_(1) << ", r_goal_(2): " << probe1.r_goal_(2) << std::endl;
                }
            }
            else if(probe1.step_ == 4)
            {
                plane_position = probe1.getLinearMotorPosition();
                if(probe1.write_flag_ == true)
                {
                    probe1.publishProbeControl(probe1.computePosture(probe1.r_goal_));
                }
                else if(abs(plane_position(0) - probe1.r_goal_(1)) < tolerance && abs(plane_position(1) - probe1.r_goal_(2)) < tolerance && probe1.moved_to_x_flag_)
                {
                    probe1.publishMovedToX();
                }
                
            }
            else if(probe1.step_ == 5)
            {
                // std::cout << "!!!!!!!!step 5!!!!!!!!!" << std::endl;
            }
        }
        ros::spinOnce();     // １回だけコールバック関数を呼び出す
        rate.sleep();        // 指定した周期でループするよう寝て待つ

    }

    return 0;
}