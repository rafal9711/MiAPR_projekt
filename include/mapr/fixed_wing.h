//
// Created by michael on 07.09.2020.
//
#pragma once

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "message_filters/subscriber.h"
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include "fcl/shape/geometric_shapes.h"
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"
#include <fcl/octree.h>
#include <fcl/collision_data.h>
#include "fcl/collision.h"
#include "fcl/collision_func_matrix.h"
#include "fcl/narrowphase/narrowphase.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
using namespace std;
using namespace fcl;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class fixed_wing {
public:
    vector<CollisionObject> collison_object();
};
