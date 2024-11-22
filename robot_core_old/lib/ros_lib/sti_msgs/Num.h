
#ifndef _ROS_sti_msgs_Num_h
#define _ROS_sti_msgs_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

namespace sti_msgs
{
  class Num : public ros::Msg
  {
    public:
	    typedef std_msgs::Int8 _age_type; _age_type age;
	    typedef std_msgs::Int32 _score_type; _score_type score;
      typedef std_msgs::String _mess_type; _mess_type mess;

    Num():
	    age(),
	    score(),
      mess()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->age.serialize(outbuffer + offset);
	    offset += this->score.serialize(outbuffer + offset);
      offset += this->mess.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->age.deserialize(inbuffer + offset);
	    offset += this->score.deserialize(inbuffer + offset);
      offset += this->mess.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "sti_msgs/Num"; };
    const char * getMD5(){ return "5b79538adb214dcea39d1b848c33ed9c"; };

  };

}
#endif