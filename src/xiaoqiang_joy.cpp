/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <std_msgs/Bool.h>
#include "bw_io/SetPins.h"
#include "bw_io/ReadPins.h"

class XiaoqiangTeleop
{
public:
  XiaoqiangTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void publish2();
  void updateOutput();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_,up_axis_,down_axis_,forward_axis_,backward_axis_,fastchange_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher barDetectFlag_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  bool up_axis_pressed_;
  bool down_axis_pressed_;
  ros::Timer timer_;
  ros::Timer timer2_;
  bool enable_updown_;

  ros::ServiceClient set_pins_client_;
  ros::ServiceClient read_pins_client_;
  int output_1_;
  int output_2_;

  bool fastchange_axis_pressed_;
  bool forward_axis_pressed_;
  bool backward_axis_pressed_;
  float last_vel_;
};

XiaoqiangTeleop::XiaoqiangTeleop():
  ph_("~"),
  linear_(1),
  angular_(2),
  deadman_axis_(1),
  fastchange_axis_(2),
  forward_axis_(3),
  backward_axis_(4),
  up_axis_(5),
  down_axis_(6),
  l_scale_(0.2),
  a_scale_(0.5),
  enable_updown_(false)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axis_fastchange", fastchange_axis_, fastchange_axis_);
  ph_.param("axis_forward", forward_axis_, forward_axis_);
  ph_.param("axis_backward", backward_axis_, backward_axis_);
  ph_.param("axis_up", up_axis_, up_axis_);
  ph_.param("axis_down", down_axis_, down_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("enable_updown", enable_updown_, enable_updown_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;
  fastchange_axis_pressed_ = false;
  forward_axis_pressed_ = false;
  backward_axis_pressed_ = false;

  up_axis_pressed_ = false;
  down_axis_pressed_ = false;

  last_vel_ = 0;

  output_1_ = 1;
  output_2_ = 1;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/bw_io/joy", 10, &XiaoqiangTeleop::joyCallback, this);

  set_pins_client_ = ph_.serviceClient<bw_io::SetPins>("/bw_io/set_pins");
  read_pins_client_ = ph_.serviceClient<bw_io::ReadPins>("/bw_io/read_pins");

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&XiaoqiangTeleop::publish, this));
  if(enable_updown_) timer2_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&XiaoqiangTeleop::publish2, this));
}

void XiaoqiangTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  deadman_pressed_ = joy->buttons[deadman_axis_-1];
  fastchange_axis_pressed_ = joy->buttons[fastchange_axis_-1];
  forward_axis_pressed_ = joy->buttons[forward_axis_-1];
  backward_axis_pressed_ = joy->buttons[backward_axis_-1];
  up_axis_pressed_ = joy->buttons[up_axis_-1];
  down_axis_pressed_ = joy->buttons[down_axis_-1];

  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_-1];
  vel.linear.x = l_scale_*joy->axes[linear_-1];
  if(forward_axis_pressed_)
  {
    vel.linear.x = vel.linear.x;
  }
  else if(backward_axis_pressed_)
  {
    vel.linear.x = - vel.linear.x;
  }

  if(fastchange_axis_pressed_)
  {
    vel.linear.x = -last_vel_;
  }
  else
  {
    last_vel_ = vel.linear.x;
  }
  last_published_ = vel;
}

void XiaoqiangTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if (deadman_pressed_)
  {
    if(forward_axis_pressed_ || backward_axis_pressed_ || fastchange_axis_pressed_ || std::fabs(last_published_.angular.z)>0.1)
    {
      vel_pub_.publish(last_published_);
      zero_twist_published_=false;
    }
    else if(!zero_twist_published_)
    {
      vel_pub_.publish(*new geometry_msgs::Twist());
      zero_twist_published_=true;
    }
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }

}

void XiaoqiangTeleop::updateOutput()
{
  bool read_pins_srv_exist = read_pins_client_.waitForExistence(ros::Duration(1)); //等待1秒
  if (read_pins_srv_exist)
  {
    bw_io::ReadPins srv;
    srv.request.method = 0;
    if(read_pins_client_.call(srv))
    {
      output_1_ = srv.response.output_buttons[0];
      output_2_ = srv.response.output_buttons[1];
    }
  }
}

void XiaoqiangTeleop::publish2()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  bool need_set = false;
  int output_1_set = 1;
  int output_2_set = 1;
  if (deadman_pressed_)
  {
    updateOutput();
    //设置升降设备
    if(down_axis_pressed_)
    {
      //1高2低对应下降
      if(output_1_ !=1 || output_2_ !=0)
      {
        output_1_set = 1;
        output_2_set = 0;
        need_set = true;
      }
    }
    else
    {
      if(up_axis_pressed_)
      {
        //1低2高对应下降
        if(output_1_ !=0 || output_2_ !=1)
        {
          output_1_set = 0;
          output_2_set = 1;
          need_set = true;
        }
      }
      else
      {
        //1高和2高对应停止
        if(output_1_ !=1 || output_2_ !=1)
        {
          output_1_set = 1;
          output_2_set = 1;
          need_set = true;
        }
      }
    }
  }

  if(need_set)
  {
    bool set_pins_srv_exist = set_pins_client_.waitForExistence(ros::Duration(1)); //等待1秒
    if (set_pins_srv_exist)
    {
      bw_io::SetPins srv;
      srv.request.set_buttons.resize(2);
      srv.request.button_value.resize(2);
      srv.request.set_buttons[0] = 1;
      srv.request.set_buttons[1] = 2;
      srv.request.button_value[0] = output_1_set;
      srv.request.button_value[1] = output_2_set;
      set_pins_client_.call(srv);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Xiaoqiang_teleop");
  XiaoqiangTeleop Xiaoqiang_teleop;

  ros::spin();
}
