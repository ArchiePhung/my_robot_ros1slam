#ifndef _ROS_sti_msgs_Vel_msg_h
#define _ROS_sti_msgs_Vel_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int16.h"

namespace sti_msgs
{
  class Vel_msg : public ros::Msg
  {
    public:
	    typedef std_msgs::Int16 _pwm_left_motor_type; _pwm_left_motor_type pwm_left_motor;
      typedef std_msgs::Int16 _pwm_right_motor_type; _pwm_right_motor_type pwm_right_motor;
      typedef std_msgs::Float32 _vel_left_motor_type; _vel_left_motor_type vel_left_motor;
      typedef std_msgs::Float32 _vel_right_motor_type; _vel_right_motor_type vel_right_motor;
      typedef std_msgs::Int8 _dir_type; _dir_type dir;
	    
    Vel_msg():
	    pwm_left_motor(),
	    pwm_right_motor(),
	    vel_left_motor(),
	    vel_right_motor(),
      dir()      
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pwm_left_motor.serialize(outbuffer + offset);
      offset += this->pwm_right_motor.serialize(outbuffer + offset);
      offset += this->vel_left_motor.serialize(outbuffer + offset);
      offset += this->vel_right_motor.serialize(outbuffer + offset);
      offset += this->dir.serialize(outbuffer + offset);      

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pwm_left_motor.deserialize(inbuffer + offset);
      offset += this->pwm_right_motor.deserialize(inbuffer + offset);
      offset += this->vel_left_motor.deserialize(inbuffer + offset);
      offset += this->vel_right_motor.deserialize(inbuffer + offset);
      offset += this->dir.deserialize(inbuffer + offset);

      return offset;
    }

    // virtual int serialize(unsigned char *outbuffer) const
    // {
    //   	int offset = 0;

    //     union {
    //       float real;
    //       uint32_t base;
    //     } u_pwm_left_motor;
    //     u_pwm_left_motor.real = this->pwm_left_motor;
    //     *(outbuffer + offset + 0) = (u_pwm_left_motor.base >> (8 * 0)) & 0xFF;
    //     *(outbuffer + offset + 1) = (u_pwm_left_motor.base >> (8 * 1)) & 0xFF;
    //     *(outbuffer + offset + 2) = (u_pwm_left_motor.base >> (8 * 2)) & 0xFF;
    //     *(outbuffer + offset + 3) = (u_pwm_left_motor.base >> (8 * 3)) & 0xFF;
    //     offset += sizeof(this->pwm_left_motor);

    //     union {
    //       float real;
    //       uint32_t base;
    //     } u_pwm_right_motor;
    //     u_pwm_right_motor.real = this->pwm_right_motor;
    //     *(outbuffer + offset + 0) = (u_pwm_right_motor.base >> (8 * 0)) & 0xFF;
    //     *(outbuffer + offset + 1) = (u_pwm_right_motor.base >> (8 * 1)) & 0xFF;
    //     *(outbuffer + offset + 2) = (u_pwm_right_motor.base >> (8 * 2)) & 0xFF;
    //     *(outbuffer + offset + 3) = (u_pwm_right_motor.base >> (8 * 3)) & 0xFF;
    //     offset += sizeof(this->pwm_right_motor);
     
    //   return offset;
    // }

    // virtual int deserialize(unsigned char *inbuffer)
    // {
    //   int offset = 0;

    //   union {
    //     float real;
    //     uint32_t base;
    //   } u_pwm_left_motor;
    //   u_pwm_left_motor.base = 0;
    //   u_pwm_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    //   u_pwm_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    //   u_pwm_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    //   u_pwm_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    //   this->pwm_left_motor = u_pwm_left_motor.real;
    //   offset += sizeof(this->pwm_left_motor);

    //   union {
    //     float real;
    //     uint32_t base;
    //   } u_pwm_right_motor;
    //   u_pwm_right_motor.base = 0;
    //   u_pwm_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    //   u_pwm_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    //   u_pwm_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    //   u_pwm_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    //   this->pwm_right_motor = u_pwm_right_motor.real;
    //   offset += sizeof(this->pwm_right_motor);

    //   return offset;
    // }
    const char * getType(){ return "message_pkg/Vel_msg"; };
    const char * getMD5(){ return "8ce996bea8b23d23b9968c0c69988650"; };

  };
}
#endif