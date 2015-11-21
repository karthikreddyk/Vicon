#pragma once
#ifndef _POSE_HPP_INCLUDED_
#define _POSE_HPP_INCLUDED_
#endif

#ifndef _STRUCTS_H_INCLUDED_
#define _STRUCTS_H_INCLUDED_
#endif                

#include <string>
#include "/home/lex/catkin_ws/src/vicon/vicon_listener/include/vicon_listener/structs.h"
//#include <vicon_listener/structs.h>

/// Structure to hold information about a single pose.
struct pose
{
  Vector3f rpy;
  facemidpts facepoints;
  //Unpack structs to C primitives
  float xtrans  /*= facepoints.x*/;
  float ytrans  /*= facepoints.y*/;
  float ztrans  /*= facepoints.z*/;

  float roll /* = rpy(0)*/;
  float pitch/* = rpy(1)*/;
  float yaw  /* = rpy(2)*/;

  //serialize the primitives
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & xtrans;
    ar & ytrans;
    ar & ztrans;

    ar & roll;
    ar & pitch;
    ar & yaw;
  }
};

