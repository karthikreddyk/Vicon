/*Olalekan Ogunmolu
SeRViCE Lab
Nov. 2015*/

#pragma once	
#ifndef _STRUCTS_H_INCLUDED_
#define _STRUCTS_H_INCLUDED_

#endif

#include <Eigen/Dense>

using namespace Eigen;

struct orth
{
	Vector3d e1;
	Vector3d e2;
	Vector3d e3;
	Vector3d e4;
};

struct facemidpts
{
    float x;
    float y;
    float z;
};

struct headmarkers
{
    geometry_msgs::Point_<std::allocator<void> > foreo;
    geometry_msgs::Point_<std::allocator<void> > lefto;
    geometry_msgs::Point_<std::allocator<void> > righto;
    geometry_msgs::Point_<std::allocator<void> > chino;
};
