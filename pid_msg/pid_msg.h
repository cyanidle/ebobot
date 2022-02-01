#ifndef _ROS_pid_msg_pid_msg_h
#define _ROS_pid_msg_pid_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pid_msg
{

  class pid_msg : public ros::Msg
  {
    public:
      typedef int32_t _mot_type;
      _mot_type mot;
      typedef float _P_type;
      _P_type P;
      typedef float _I_type;
      _I_type I;
      typedef float _D_type;
      _D_type D;

    pid_msg():
      mot(0),
      P(0),
      I(0),
      D(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_mot;
      u_mot.real = this->mot;
      *(outbuffer + offset + 0) = (u_mot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mot);
      union {
        float real;
        uint32_t base;
      } u_P;
      u_P.real = this->P;
      *(outbuffer + offset + 0) = (u_P.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_P.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_P.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_P.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->P);
      union {
        float real;
        uint32_t base;
      } u_I;
      u_I.real = this->I;
      *(outbuffer + offset + 0) = (u_I.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_I.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_I.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_I.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->I);
      union {
        float real;
        uint32_t base;
      } u_D;
      u_D.real = this->D;
      *(outbuffer + offset + 0) = (u_D.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_D.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_D.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_D.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->D);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_mot;
      u_mot.base = 0;
      u_mot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mot = u_mot.real;
      offset += sizeof(this->mot);
      union {
        float real;
        uint32_t base;
      } u_P;
      u_P.base = 0;
      u_P.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_P.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_P.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_P.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->P = u_P.real;
      offset += sizeof(this->P);
      union {
        float real;
        uint32_t base;
      } u_I;
      u_I.base = 0;
      u_I.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_I.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_I.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_I.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->I = u_I.real;
      offset += sizeof(this->I);
      union {
        float real;
        uint32_t base;
      } u_D;
      u_D.base = 0;
      u_D.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_D.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_D.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_D.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->D = u_D.real;
      offset += sizeof(this->D);
     return offset;
    }

    virtual const char * getType() override { return "pid_msg/pid_msg"; };
    virtual const char * getMD5() override { return "c2cb38cf9fe74a377ee3b81934555010"; };

  };

}
#endif
