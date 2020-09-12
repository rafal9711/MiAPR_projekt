//
// Created by michael on 07.09.2020.
//

#pragma once
#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include "fcl/shape/geometric_shapes.h"
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"
#include <fcl/octree.h>
#include <fcl/collision_data.h>
#include "fcl/collision.h"
#include "fcl/collision_func_matrix.h"
#include "fcl/narrowphase/narrowphase.h"
#include <ompl/config.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace check_validator {
    class Planner {
    public:
        ros::NodeHandle node;

        ros::Publisher visPub;
        ros::Publisher visCubePub;
        ros::Publisher visCylinderPub;

        void drawSphere(fcl::Vec3f vector, float radius);
        void drawCUBE(fcl::Vec3f vector , int id , int c_color, float x, float y, float z);
        void drawCYLINDER(fcl::Vec3f vec ,fcl::Vec3f rot, int id , int c_color,float radius, float lz);

        /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
        Planner(ros::NodeHandle& _nodeHandle);

        /*!
       * plan path
       */
        nav_msgs::Path planPath();

        static void setTransform(std::vector<fcl::CollisionObject>& vector, const ob::RealVectorStateSpace::StateType *pos, const ob::SO3StateSpace::StateType *rot);

        std::vector<fcl::CollisionObject> vec;

    private:
        /// node handle
        ros::NodeHandle& nodeHandle;
        /// bounds for the x axis
        std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

        /// bounds for the y axis
        std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

        /// start position
//        std::shared_ptr<ompl::base::ScopedState<>> start;

        /// search space
        std::shared_ptr<ompl::base::StateSpace> space;

        ob::SpaceInformationPtr si;

        // create a problem instance
        ob::ProblemDefinitionPtr pdef;

        /// configure node
        void configure(void);

        /// extract path
        nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);

        std::shared_ptr<fcl::CollisionGeometry> tree_obj;

        bool isStateValid(const ob::State *state);

        ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
    };
}