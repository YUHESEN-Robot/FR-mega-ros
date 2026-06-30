#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

namespace yhs_tool
{

  CanControl::CanControl()
  {
    ros::NodeHandle private_node("~");

    std::string ultrasonic_numbers_str;
    private_node.getParam("ultrasonic_number", ultrasonic_numbers_str);
    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
  	private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
  	private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
  	private_node.param("/yhs_can_control/wheel_base", wheel_base_, 0.6);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &CanControl::ImuDataCallBack, this);

    ebox_state_fb_pub_ = nh_.advertise<yhs_can_msgs::Ebox_State_fb>("ebox_state_fb", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
    last_imu_time_ = ros::Time(0);
    

    std::istringstream iss(ultrasonic_numbers_str);
    int number;
    while (iss >> number)
    {
      ultrasonic_number_.push_back(number);
    }
  }

  CanControl::~CanControl()
  {
  }

  // io控制回调函数
  void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
  {
    const bool enable = msg.io_cmd_enable;
    const bool lower_beam = msg.io_cmd_lower_beam_headlamp;
    const bool upper_beam = msg.io_cmd_upper_beam_headlamp;
    const unsigned char turn_lamp = msg.io_cmd_turn_lamp; 
    const bool braking_lamp = msg.io_cmd_braking_lamp;
    const bool clearance_lamp = msg.io_cmd_clearance_lamp;
    const bool fog_lamp = msg.io_cmd_fog_lamp;
    const bool speaker = msg.io_cmd_speaker;
    const int disCharge = msg.io_cmd_disCharge;

    static unsigned char count_1 = 0;

    cmd_mutex_.lock();

    memset(sendData_u_io_, 0, 8);

    // Byte 0
    sendData_u_io_[0] = enable;

    if (lower_beam)     sendData_u_io_[1] |= 0x01;
    if (upper_beam)     sendData_u_io_[1] |= 0x02;
    sendData_u_io_[1] |= (turn_lamp << 2); 
    if (braking_lamp)   sendData_u_io_[1] |= 0x10;
    if (clearance_lamp) sendData_u_io_[1] |= 0x20;
    if (fog_lamp)       sendData_u_io_[1] |= 0x40;
    
    // Byte 2
    sendData_u_io_[2] = speaker;

    // Byte 5
    sendData_u_io_[5] = disCharge;

    // Byte 6
    count_1++;
    if (count_1 == 16) count_1 = 0;
    sendData_u_io_[6] = count_1 << 4;

    // Byte 7
    sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

    // 发送
    send_frames_[0].can_id = 0x18C4D7D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_io_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0)
    {
      ROS_ERROR("send io_cmd failed, error code: %d", ret);
    }

    cmd_mutex_.unlock();
  }

  // 速度闭环控制
  void CanControl::auto_spd_ctrl_cmdCallBack(const yhs_can_msgs::auto_spd_ctrl_cmd msg)
  {
    const unsigned short vel = (msg.ctrl_cmd_velocity < 0) ? 0 : (unsigned short)(msg.ctrl_cmd_velocity * 1000);
    const short angular = msg.ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;
    const unsigned char brake = msg.ctrl_cmd_Brake;
    
    static unsigned char count = 0;

    cmd_mutex_.lock();
    memset(sendData_u_vel_, 0, 8);
    
    sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & gear);
    sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));
    
    sendData_u_vel_[1] = (vel >> 4) & 0xff;
    
    sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));
    sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));
    
    sendData_u_vel_[3] = (angular >> 4) & 0xff;

    sendData_u_vel_[4] |= (0x0f & (angular >> 12));
    sendData_u_vel_[4] |= (0xf0 & ((brake & 0x0f) << 4));

    sendData_u_vel_[5] = (brake >> 4) & 0x0f;

    count++;
    if (count == 16)
    count = 0;
    sendData_u_vel_[6] = count << 4;

    sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

    send_frames_[0].can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_vel_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0)
    {
      ROS_ERROR("send message failed, error code: %d", ret);
    }

    cmd_mutex_.unlock();
  }

  // 扭矩闭环控制
  void CanControl::auto_Torque_ctrl_cmdCallBack(const yhs_can_msgs::auto_Torque_ctrl_cmd msg)
  {
    const short angular = msg.ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;
    const unsigned char acc_running_mode = msg.ctrl_cmd_acc_running_mode;
    const unsigned char energy_recovery_mode = msg.ctrl_cmd_energy_recovery_mode;
    const unsigned char acc_pedal_opening = msg.ctrl_cmd_acc_pedal_opening;
    const unsigned char brake = msg.ctrl_cmd_Brake;
    
    static unsigned char count = 0;

    cmd_mutex_.lock();

    memset(sendData_u_vel_, 0, 8);

    sendData_u_vel_[0] |= (0x0f & gear);
    sendData_u_vel_[0] |= ((0x03 & acc_running_mode) << 4);
    sendData_u_vel_[0] |= ((0x03 & energy_recovery_mode) << 6);

    sendData_u_vel_[1] = acc_pedal_opening;

    sendData_u_vel_[2] = angular & 0xff;

    sendData_u_vel_[3] = (angular >> 8) & 0xff;

    sendData_u_vel_[4] = brake;
    
    count++;
    if (count == 16) count = 0;
    sendData_u_vel_[6] = count << 4;

    sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

    send_frames_[0].can_id = 0x18C4D1D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_vel_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0)
    {
      ROS_ERROR("send auto_Torque_ctrl_cmd failed, error code: %d", ret);
    }

    cmd_mutex_.unlock();
  }

  // remote_Torque_ctrl_cmd
  void CanControl::remote_Torque_ctrl_cmdCallBack(const yhs_can_msgs::remote_Torque_ctrl_cmd msg)
  {
    const short angular = msg.ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;
    const unsigned char acc_running_mode = msg.ctrl_cmd_acc_running_mode;
    const unsigned char energy_recovery_mode = msg.ctrl_cmd_energy_recovery_mode;
    const unsigned char acc_pedal_opening = msg.ctrl_cmd_acc_pedal_opening;
    const unsigned char brake = msg.ctrl_cmd_Brake;
    
    static unsigned char count = 0;

    cmd_mutex_.lock();

    memset(sendData_u_vel_, 0, 8);

    sendData_u_vel_[0] |= (0x0f & gear);
    sendData_u_vel_[0] |= ((0x03 & acc_running_mode) << 4);
    sendData_u_vel_[0] |= ((0x03 & energy_recovery_mode) << 6);

    sendData_u_vel_[1] = acc_pedal_opening;

    sendData_u_vel_[2] = angular & 0xff;

    sendData_u_vel_[3] = (angular >> 8) & 0xff;

    sendData_u_vel_[4] = brake;
    
    count++;
    if (count == 16) count = 0;
    sendData_u_vel_[6] = count << 4;

    sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

    send_frames_[0].can_id = 0x18C4D0D0  | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_vel_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0)
    {
      ROS_ERROR("send auto_Torque_ctrl_cmd failed, error code: %d", ret);
    }

    cmd_mutex_.unlock();
  }

  // 数据接收解析线程
  void CanControl::recvData()
  {
    while (ros::ok())
    {
      if (read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
      {
        for (int j = 0; j < 1; j++)
        {
          switch (recv_frames_[0].can_id)
          {
            // 速度控制反馈
            case 0x18C4D2EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::ctrl_fb msg;
              msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
              
              // 速度解析
              msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
              
              // 转向解析
              msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

              msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0xf0) >> 4 | (recv_frames_[0].data[5] & 0x0f) << 4;
              msg.ctrl_fb_mode = (recv_frames_[0].data[5] & 0x30) >> 4;
              msg.ctrl_fb_acc_running_mode = recv_frames_[0].data[6] & 0x30;
              msg.ctrl_fb_energy_recovery_mode = (recv_frames_[0].data[6] & 0x0c) >> 2;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                ctrl_fb_pub_.publish(msg);

                // 里程计解算逻辑 (包含倒档处理)
                float current_vel = msg.ctrl_fb_velocity;
                if (msg.ctrl_fb_gear == 2) 
                {
                    current_vel = -current_vel;
                }
                float steering_radian = msg.ctrl_fb_steering * 3.1415926 / 180.0;

                OdomPub(current_vel, steering_radian); 
              }
              break;
            }

            // 左轮反馈
            case 0x18C4D7EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::lr_wheel_fb msg;
              msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
              msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                lr_wheel_fb_pub_.publish(msg);
              }
              break;
            }

            // 右轮反馈
            case 0x18C4D8EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::rr_wheel_fb msg;
              msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
              msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                rr_wheel_fb_pub_.publish(msg);
              }
              break;
            }

            // io反馈 (重构：使用直接位运算赋值)
            case 0x18C4DAEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::io_fb msg;
              
              // Byte 0
              msg.io_fb_enable = (recv_frames_[0].data[0] & 0x01);

              // Byte 1 (灯光)
              msg.io_fb_lower_beam_headlamp = (recv_frames_[0].data[1] & 0x01);
              msg.io_fb_upper_beam_headlamp = (recv_frames_[0].data[1] & 0x02);
              msg.io_fb_turn_lamp           = (recv_frames_[0].data[1] & 0x0c) >> 2;
              msg.io_fb_braking_lamp        = (recv_frames_[0].data[1] & 0x10);
              msg.io_fb_clearance_lamp      = (recv_frames_[0].data[1] & 0x20);
              msg.io_fb_fog_lamp            = (recv_frames_[0].data[1] & 0x40);

              // Byte 2 (喇叭)
              msg.io_fb_speaker             = (recv_frames_[0].data[2] & 0x01);

              // Byte 3 (防撞条)
              msg.io_fb_fl_impact_sensor    = (recv_frames_[0].data[3] & 0x01);
              msg.io_fb_fm_impact_sensor    = (recv_frames_[0].data[3] & 0x02);
              msg.io_fb_fr_impact_sensor    = (recv_frames_[0].data[3] & 0x04);
              msg.io_fb_rl_impact_sensor    = (recv_frames_[0].data[3] & 0x08);
              msg.io_fb_rm_impact_sensor    = (recv_frames_[0].data[3] & 0x10);
              msg.io_fb_rr_impact_sensor    = (recv_frames_[0].data[3] & 0x20);

              // Byte 4 (跌落传感器)
              msg.io_fb_fl_drop_sensor      = (recv_frames_[0].data[4] & 0x01);
              msg.io_fb_fm_drop_sensor      = (recv_frames_[0].data[4] & 0x02);
              msg.io_fb_fr_drop_sensor      = (recv_frames_[0].data[4] & 0x04);
              msg.io_fb_rl_drop_sensor      = (recv_frames_[0].data[4] & 0x08);
              msg.io_fb_rm_drop_sensor      = (recv_frames_[0].data[4] & 0x10);
              msg.io_fb_rr_drop_sensor      = (recv_frames_[0].data[4] & 0x20);

              // Byte 5 & Byte 1 (充电相关)
              msg.io_fb_disCharge           = (recv_frames_[0].data[5] & 0x01);
              msg.io_fb_chargeEn            = (recv_frames_[0].data[5] & 0x02);

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                io_fb_pub_.publish(msg);
              }
              break;
            }

            // 里程计反馈
            case 0x18C4DEEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::odo_fb msg;
              msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
              msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

              odo_fb_pub_.publish(msg);
              break;
            }

            // bms_Infor反馈
            case 0x18C4E1EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::bms_Infor_fb msg;
              msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;
              msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;
              msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                bms_Infor_fb_pub_.publish(msg);
              }
              break;
            }

            // bms_flag_Infor反馈 (重构：使用直接位运算赋值)
            case 0x18C4E2EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::bms_flag_Infor_fb msg;
              msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

              // Byte 1
              msg.bms_flag_Infor_single_ov      = (recv_frames_[0].data[1] & 0x01);
              msg.bms_flag_Infor_single_uv      = (recv_frames_[0].data[1] & 0x02);
              msg.bms_flag_Infor_ov             = (recv_frames_[0].data[1] & 0x04);
              msg.bms_flag_Infor_uv             = (recv_frames_[0].data[1] & 0x08);
              msg.bms_flag_Infor_charge_ot      = (recv_frames_[0].data[1] & 0x10);
              msg.bms_flag_Infor_charge_ut      = (recv_frames_[0].data[1] & 0x20);
              msg.bms_flag_Infor_discharge_ot   = (recv_frames_[0].data[1] & 0x40);
              msg.bms_flag_Infor_discharge_ut   = (recv_frames_[0].data[1] & 0x80);

              // Byte 2
              msg.bms_flag_Infor_charge_oc      = (recv_frames_[0].data[2] & 0x01);
              msg.bms_flag_Infor_discharge_oc   = (recv_frames_[0].data[2] & 0x02);
              msg.bms_flag_Infor_short          = (recv_frames_[0].data[2] & 0x04);
              msg.bms_flag_Infor_ic_error       = (recv_frames_[0].data[2] & 0x08);
              msg.bms_flag_Infor_lock_mos       = (recv_frames_[0].data[2] & 0x10);
              
              msg.bms_flag_info_charge_state    = (recv_frames_[0].data[2] & 0x60) >> 5;

              // 预留
              msg.reserved = ((recv_frames_[0].data[2] & 0x80) >> 7 ) | ( (recv_frames_[0].data[3] & 0x0f) << 1) ;

              msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;
              msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                bms_flag_Infor_fb_pub_.publish(msg);
              }
              break;
            }

            // Drive_fb_MCUEcoder反馈
            case 0x18C4DCEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::Drive_MCUEcoder_fb msg;
              msg.Drive_fb_MCUEcoder = (int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]);
              msg.Drive_fb_MCUTorque = (int)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4]);

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                Drive_MCUEcoder_fb_pub_.publish(msg);
              }
              break;
            }

            // Veh_fb_Diag反馈
            case 0x18C4EAEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::Veh_Diag_fb msg;
              msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

              // Byte 0
              msg.Veh_fb_AutoSpdCANCtrlCmd    = (recv_frames_[0].data[0] & 0x10);
              msg.Veh_fb_AutoIOCANCmd         = (recv_frames_[0].data[0] & 0x20);
              msg.Veh_fb_RemoteTrqCANCtrlCmd  = (recv_frames_[0].data[0] & 0x40);
              msg.Veh_fb_AutoTrqCANCtrlCmd    = (recv_frames_[0].data[0] & 0x80);

              // Byte 1 (EPS)
              msg.Veh_fb_EPSDisOnline         = (recv_frames_[0].data[1] & 0x01);
              msg.Veh_fb_EPSfault             = (recv_frames_[0].data[1] & 0x02);
              msg.Veh_fb_EPSMosfetOT          = (recv_frames_[0].data[1] & 0x04);
              msg.Veh_fb_EPSWarning           = (recv_frames_[0].data[1] & 0x08);
              msg.Veh_fb_EPSDisWork           = (recv_frames_[0].data[1] & 0x10);
              msg.Veh_fb_EPSOverCurrent       = (recv_frames_[0].data[1] & 0x20);

              // Byte 2 & 3 (EHB)
              msg.Veh_fb_EHBMCUFaultFb = ((recv_frames_[0].data[3] << 4) | ((recv_frames_[0].data[2] & 0xf0) >> 4));

              // Byte 4 & 5 (DrvMCU)
              msg.Veh_fb_DrvMCUFaultFb = ((recv_frames_[0].data[5] & 0x0f) << 8) | (recv_frames_[0].data[4]);

              // Byte 5 (AUX)
              msg.Veh_fb_AUXBMSDisOnline      = (recv_frames_[0].data[5] & 0x10);
              msg.Veh_fb_AuxScram             = (recv_frames_[0].data[5] & 0x20);
              msg.Veh_fb_AuxRemoteClose       = (recv_frames_[0].data[5] & 0x40);
              msg.Veh_fb_AuxRemoteDisOnline   = (recv_frames_[0].data[5] & 0x80);

              // 辅件故障预留
              msg.Veh_fb_AuxReserve = recv_frames_[0].data[6] & 0x0f;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                Veh_Diag_fb_pub_.publish(msg);
              }
              break;
            }

            // ultrasonic 
            static unsigned short ultra_data[8] = {0};
            case 0x18C4E8EF | CAN_EFF_FLAG:
            {
              ultra_data[0] = (unsigned short)((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]);
              ultra_data[1] = (unsigned short)(recv_frames_[0].data[2] << 4 | ((recv_frames_[0].data[1] & 0xf0) >> 4));
              ultra_data[2] = (unsigned short)((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]);
              ultra_data[3] = (unsigned short)(recv_frames_[0].data[5] << 4 | ((recv_frames_[0].data[4] & 0xf0) >> 4));
              break;
            }

            case 0x18C4E9EF | CAN_EFF_FLAG:
            {
              ultra_data[4] = (unsigned short)((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]);
              ultra_data[5] = (unsigned short)(recv_frames_[0].data[2] << 4 | ((recv_frames_[0].data[1] & 0xf0) >> 4));
              ultra_data[6] = (unsigned short)((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]);
              ultra_data[7] = (unsigned short)(recv_frames_[0].data[5] << 4 | ((recv_frames_[0].data[4] & 0xf0) >> 4));

              // 超声波雷达调试输出信息
              // std::cout << "ultra_data[0]: " << ultra_data[0] << std::endl;
              // std::cout << "ultra_data[1]: " << ultra_data[1] << std::endl;
              // std::cout << "ultra_data[2]: " << ultra_data[2] << std::endl;
              // std::cout << "ultra_data[3]: " << ultra_data[3] << std::endl;
              // std::cout << "ultra_data[4]: " << ultra_data[4] << std::endl;
              // std::cout << "ultra_data[5]: " << ultra_data[5] << std::endl;
              // std::cout << "ultra_data[6]: " << ultra_data[6] << std::endl;
              // std::cout << "ultra_data[7]: " << ultra_data[7] << std::endl;


              yhs_can_msgs::ultrasonic ultra_msg;
              ultra_msg.ultrasonic_fb_01 = ultra_data[ultrasonic_number_[0]];
              ultra_msg.ultrasonic_fb_02 = ultra_data[ultrasonic_number_[1]];
              ultra_msg.ultrasonic_fb_03 = ultra_data[ultrasonic_number_[2]];
              ultra_msg.ultrasonic_fb_04 = ultra_data[ultrasonic_number_[3]];
              ultra_msg.ultrasonic_fb_05 = ultra_data[ultrasonic_number_[4]];
              ultra_msg.ultrasonic_fb_06 = ultra_data[ultrasonic_number_[5]];
              ultra_msg.ultrasonic_fb_07 = ultra_data[ultrasonic_number_[6]];
              ultra_msg.ultrasonic_fb_08 = ultra_data[ultrasonic_number_[7]];

              ultrasonic_pub_.publish(ultra_msg);
              break;
            }

            static yhs_can_msgs::Ebox_State_fb ebox_msg;
              // 1. 诊断状态 (0x18FFFFFB)
            case 0x18FFFFFB | CAN_EFF_FLAG:
            {
              ebox_msg.diag_ebox_chg_mos_h_tem_limt   = (recv_frames_[0].data[0] & 0x01);
              ebox_msg.diag_ebox_chg_mos_h_tem_close  = (recv_frames_[0].data[0] & 0x02);
              ebox_msg.diag_ebox_ebox_mos_h_tem_limt  = (recv_frames_[0].data[0] & 0x04);
              ebox_msg.diag_ebox_ebox_mos_h_tem_close = (recv_frames_[0].data[0] & 0x08);
              
              ebox_msg.diag_ebox_h_light_fault        = (recv_frames_[0].data[1] & 0x01);
              ebox_msg.diag_ebox_b_light_fault        = (recv_frames_[0].data[1] & 0x02);
              ebox_msg.diag_ebox_l_light_fault        = (recv_frames_[0].data[1] & 0x04);
              ebox_msg.diag_ebox_r_light_fault        = (recv_frames_[0].data[1] & 0x08);
              break;
            }

            // 2. 48V 电压电流 (0x18FFFFFC)
            case 0x18FFFFFC | CAN_EFF_FLAG:
            {
              ebox_msg.mcu_48v_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) * 0.1;
              ebox_msg.mcu_current     = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) * 0.1;
              ebox_msg.ebox_current    = (float)((short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) * 0.1;
              ebox_msg.cus_48v_voltage = (float)((unsigned short)(recv_frames_[0].data[7] << 8 | recv_frames_[0].data[6])) * 0.1;
              break;
            }

            // 3. 辅助电源与温度 (0x18FFFFFD) -> 作为最后一帧进行发布
            case 0x18FFFFFD | CAN_EFF_FLAG:
            {
              ebox_msg.cus_24v_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) * 0.1;
              ebox_msg.cus_12v_voltage = (float)((unsigned short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) * 0.1;
              ebox_msg.bat_48v_voltage = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) * 0.1;
              
              // 温度有偏移量 -50
              ebox_msg.ebox_mos_tem = (float)((int)recv_frames_[0].data[6] - 50);
              ebox_msg.chg_mos_tem  = (float)((int)recv_frames_[0].data[7] - 50);

              // 收到这一帧时，认为一组数据更新完毕，进行发布
              ebox_msg.header.stamp = ros::Time::now();
              ebox_msg.header.frame_id = baseFrame_; // 或者用 odomFrame_，通常状态信息跟base_link走
              
              ebox_state_fb_pub_.publish(ebox_msg);
              break;
            }

            default:
              break;
          }
        }
      }
    }
  }

  void CanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
      std::lock_guard<std::mutex> lock(mutex_);

      last_imu_time_ = ros::Time::now(); 

      tf2::Quaternion quaternion;
      tf2::fromMsg(imu_data_msg->orientation, quaternion);
      tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void CanControl::OdomPub(const float velocity, const float steering)
  {
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0; 

    double x_mid = 0.0;
    double y_mid = 0.0;

    static tf2_ros::TransformBroadcaster odom_broadcaster;
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();

    double vx = velocity;
    double vth = vx * tan(steering) / wheel_base_;
    
    double dt = (current_time - last_time).toSec();
    bool is_imu_active = (current_time - last_imu_time_).toSec() < 0.2;
    if (is_imu_active)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        th = imu_yaw_; 
    }
    else
    {
        th += vth * dt; 
    }
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;

    x += delta_x;
    y += delta_y;
    x_mid = x + wheel_base_ / 2 * cos(th);
    y_mid = y + wheel_base_ / 2 * sin(th);

    tf2::Quaternion quat;
    if (is_imu_active) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat.setRPY(imu_roll_, imu_pitch_, th);
    } else {
        quat.setRPY(0, 0, th);
    }
    
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odomFrame_;
    odom_trans.child_frame_id = baseFrame_;

    odom_trans.transform.translation.x = x_mid;
    odom_trans.transform.translation.y = y_mid;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    if (tfUsed_)
        odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odomFrame_;

    odom.pose.pose.position.x = x_mid;
    odom.pose.pose.position.y = y_mid;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = baseFrame_;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    
    odom.twist.twist.angular.z = vth; 

    // 协方差矩阵设置 (保持原样)
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[35] = 0.2;
    odom.pose.covariance[14] = 1e10;
    odom.pose.covariance[21] = 1e10;
    odom.pose.covariance[28] = 1e10;

    odom_pub_.publish(odom);

    last_time = current_time;
  }

  // 数据发送线程
  void CanControl::sendData()
  {
    ros::Rate loop(100);

    while (ros::ok())
    {

      loop.sleep();
    }
  }

  void CanControl::run()
  {

    remote_Torque_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::remote_Torque_ctrl_cmd>("remote_Torque_ctrl_cmd", 5, &CanControl::remote_Torque_ctrl_cmdCallBack, this);
    auto_Torque_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::auto_Torque_ctrl_cmd>("auto_Torque_ctrl_cmd", 5, &CanControl::auto_Torque_ctrl_cmdCallBack, this);
    auto_spd_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::auto_spd_ctrl_cmd>("auto_spd_ctrl_cmd", 5, &CanControl::auto_spd_ctrl_cmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);

    ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb", 5);
    lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb", 5);
    rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb", 5);
    io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb", 5);
    odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb", 5);
    bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb", 5);
    bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb", 5);
    Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb", 5);
    Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic", 5);

    // 打开设备
    dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (dev_handler_ < 0)
    {
      ROS_ERROR(">>open can deivce error!");
      return;
    }
    else
    {
      ROS_INFO(">>open can deivce success!");
    }

    struct ifreq ifr;

    std::string can_name("can0");

    strcpy(ifr.ifr_name, can_name.c_str());

    ioctl(dev_handler_, SIOCGIFINDEX, &ifr);

    // bind socket to network interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (ret < 0)
    {
      ROS_ERROR(">>bind dev_handler error!\r\n");
      return;
    }

    // 创建接收发送数据线程
    boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
    //	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

    ros::spin();

    close(dev_handler_);
  }

}

// 主函数
int main(int argc, char **argv)
{
  ros::init(argc, argv, "yhs_can_control_node");

  yhs_tool::CanControl cancontrol;
  cancontrol.run();

  return 0;
}
