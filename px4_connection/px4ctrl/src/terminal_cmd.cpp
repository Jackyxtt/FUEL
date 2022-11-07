#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/RCIn.h>
#include "KeyboardEvent.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/PositionTarget.h>
#include "Eigen/Core"


using namespace std;

mavros_msgs::RCIn msg;//即将发布的遥控器指令
mavros_msgs::CommandBool arm_cmd;//即将调用解锁service
quadrotor_msgs::TakeoffLand TakeoffAndLand_cmd;//即将发布的起降指令
mavros_msgs::SetMode offboard_cmd;//切换offboard命令
mavros_msgs::PositionTarget takeoff_cmd;


ros::Publisher move_pub;
//callback function
ros::ServiceClient arm_call;
ros::ServiceClient offboard_call;
ros::Publisher takeoff_pub;
ros::Publisher setpoint_raw_local_pub;
ros::Subscriber position_target_sub;
Eigen::Vector3d pos_drone_fcu_target, vel_drone_fcu_target, accel_drone_fcu_target;

void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
}
//mian function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_cmd");
    ros::NodeHandle nh("~");
    KeyboardEvent keyboardcontrol;
    char key_now, key_wait;
    char key_last;
    string uav_name;
    // nh.param<string>("uav_name", uav_name, "/uav0");
    // if (uav_name == "/uav0")
    //     uav_name = "";

    //　【发布】　控制指令
    move_pub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in", 10);
    // arm_call = nh.advertiseService
    takeoff_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/uav1/px4ctrl/takeoff_land", 10);
    arm_call = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    offboard_call = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);

    // 检查飞控能否收到控制量
    position_target_sub = nh.subscribe<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/target_local", 10, pos_target_cb);


    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>> Terminal Control <<<<<<<<<<<<<<<<<<<<<<<<<<"<< endl;
    cout << "ENTER key to control the drone: " <<endl;
    cout << "Space for Takeoff, L for Land, 0 for init, R for runing FUEL (double press H for exit), 5 for hover, 6 for cmd\n 9 for arm and offboard" <<endl;
    cout << "Move mode is fixed (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" <<endl;
    cout << "CTRL-C to quit." <<endl;

    while (ros::ok())
    {
      nh.getParam("uav_name", uav_name);
      keyboardcontrol.RosWhileLoopRun();
      key_now = keyboardcontrol.GetPressedKey();

      // # RAW RC input state

      // std_msgs/Header header
      // uint8 rssi
      // uint16[] channels

      switch (key_now)
      {
        
        case U_KEY_0:{
          cout << " " <<endl;
          cout << "Init px4ctrl." <<endl;
          // 初始化命令
          msg.header.stamp = ros::Time::now();
          msg.rssi = 0;
          msg.channels = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
          move_pub.publish(msg);

          sleep(1.0);

          break;
        }

        case U_KEY_5:{
          cout << " " <<endl;
          cout << "Switch to hover Mode." <<endl;
          msg.header.stamp = ros::Time::now();
          msg.rssi = 0;
          msg.channels = {1000, 1000, 1000, 1000, 2000, 1000, 1000, 1000};
          move_pub.publish(msg);

          sleep(1.0);

          break;
        }

        case U_KEY_6:{
          cout << " " <<endl;
          cout << "Switch to auto hover and command control Mode." <<endl;

          msg.header.stamp = ros::Time::now();
          msg.rssi = 0;
          msg.channels = {1000, 1000, 1000, 1000, 2000, 2000, 1000, 1000};

          move_pub.publish(msg);

          sleep(1.0);

          break;
        }


        // case U_KEY_1:{
        //   cout << " " <<endl;
        //   cout << "Arming" <<endl;
        //   arm_cmd.request.value = 1;
        //   arm_call.call(arm_cmd);
        //   break;
        // }

        case U_KEY_SPACE:{
          cout << " " <<endl;
          cout << "Take off" <<endl;
          takeoff_cmd.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw 0b100111111000
          takeoff_cmd.coordinate_frame = 1;
          takeoff_cmd.position.x = 0;
          takeoff_cmd.position.y = 0;
          takeoff_cmd.position.z = 0.5;
          takeoff_cmd.yaw_rate = 0;
          setpoint_raw_local_pub.publish(takeoff_cmd);
         
          // 检查飞控是否收到控制量
          cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
          cout << "Pos_target [X Y Z] : " << pos_drone_fcu_target[0] << " [ m ] "<< pos_drone_fcu_target[1]<<" [ m ] "<<pos_drone_fcu_target[2]<<" [ m ] "<<endl;
          // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
          sleep(1.0);
          break;
        }

        case U_KEY_9:{
          cout << " " <<endl;
          cout << "switch to arm and offboard" <<endl;
          offboard_cmd.request.custom_mode = "OFFBOARD";
          offboard_call.call(offboard_cmd);
          arm_cmd.request.value = true;
          arm_call.call(arm_cmd); 
        }
      }
    }
    ros::spinOnce();
    return 0;
}