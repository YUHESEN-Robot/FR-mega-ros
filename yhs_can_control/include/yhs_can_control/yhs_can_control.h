#ifndef __PIDCONTROL_NODE_H__
#define __PIDCONTROL_NODE_H__

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/auto_spd_ctrl_cmd.h"
#include "yhs_can_msgs/auto_Torque_ctrl_cmd.h"
#include "yhs_can_msgs/remote_Torque_ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor_fb.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"
#include "yhs_can_msgs/ultrasonic.h"
#include "yhs_can_msgs/Ebox_State_fb.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <sstream>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace yhs_tool
{
  class CanControl
  {
  public:
    CanControl();
    ~CanControl();

    void run();

  private:
    ros::NodeHandle nh_;

    std::string odomFrame_, baseFrame_;
  	bool tfUsed_;
  	std::string if_name_;
  	double wheel_base_;

    ros::Publisher ctrl_fb_pub_;
    ros::Publisher lr_wheel_fb_pub_;
    ros::Publisher rr_wheel_fb_pub_;
    ros::Publisher io_fb_pub_;
    ros::Publisher odo_fb_pub_;
    ros::Publisher bms_Infor_fb_pub_;
    ros::Publisher bms_flag_Infor_fb_pub_;
    ros::Publisher Drive_MCUEcoder_fb_pub_;
    ros::Publisher Veh_Diag_fb_pub_;
    ros::Publisher ultrasonic_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher ebox_state_fb_pub_;

    ros::Subscriber auto_spd_ctrl_cmd_sub_;
    ros::Subscriber auto_Torque_ctrl_cmd_sub_;
    ros::Subscriber remote_Torque_ctrl_cmd_sub_;
    ros::Subscriber io_cmd_sub_;
    ros::Subscriber imu_sub_;

    ros::Time last_imu_time_;

    boost::mutex cmd_mutex_;
    std::mutex mutex_;

    double imu_roll_ = 0.0;
    double imu_pitch_ = 0.0;
    double imu_yaw_ = 0.0;

    unsigned char sendData_u_io_[8] = {0};
    unsigned char sendData_u_vel_[8] = {0};

    int dev_handler_;
    can_frame send_frames_[2];
    can_frame recv_frames_[1];

    std::vector<int> ultrasonic_number_;

    void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
    void auto_spd_ctrl_cmdCallBack(const yhs_can_msgs::auto_spd_ctrl_cmd msg);
    void auto_Torque_ctrl_cmdCallBack(const yhs_can_msgs::auto_Torque_ctrl_cmd msg);
    void remote_Torque_ctrl_cmdCallBack(const yhs_can_msgs::remote_Torque_ctrl_cmd msg);
    void ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg);
    void OdomPub(const float velocity, const float steering);

    void recvData();
    void sendData();
  };

}

#endif
