#ifndef KVH1750TYPES_HPP
#define KVH1750TYPES_HPP

#include <base/time.h>

struct fog_acceleration
{
base::Time time;
double x;
double y;
double z;
};


struct fog_rotation_delta
{
base::Time time;
double pitch;
double roll;
double yaw;
};


struct fog_simple_orientation
{
base::Time time;
double pitch;
double roll;
double yaw;
};

#endif
