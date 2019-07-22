#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define Xposition 0x01
#define Yposition 0x02
#define Zposition 0x04

#define Zrotation 0x10
#define Xrotation 0x20
#define Yrotation 0x40

typedef struct
{
  float x, y, z;
} OFFSET;

// typedef struct JOINT;

typedef struct JOINT
{
  float x, y, z;

  const char* name = NULL;
  JOINT* parent = NULL;
  OFFSET offset;
  unsigned int num_channels = 0;
  short* channel_order = NULL;
  std::vector<JOINT*> children;
  Eigen::Matrix4d matrix;
  unsigned int channel_start = 0;
};

typedef struct
{
  JOINT* rootJoint;
  int num_channels;
} HIERARCHY;

typedef struct
{
  unsigned int num_frames;
  unsigned int num_motion_channels = 0;
  float* data = NULL;
  unsigned* joint_channel_offsets;
} MOTION;
