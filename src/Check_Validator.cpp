//
// Created by michael on 07.09.2020.
//

#include "../include/ompl_example_2d/Check_Validator.h"
#include "../include/mapr/fixed_wing.h"
#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/SimpleSetup.h>
#include <iostream>
#include <memory>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/math/transform.h"

namespace check_validator {

    Planner::Planner(ros::NodeHandle& _nodeHandle)
            : nodeHandle(_nodeHandle)
    {
        ROS_INFO("Controlling UR node started.");
        visPub           = node.advertise<visualization_msgs::Marker>("Sphere", 1);
        visCubePub       = node.advertise<visualization_msgs::Marker>("Cube", 1);
        visCylinderPub   = node.advertise<visualization_msgs::Marker>("Cylinder", 1);

        fixed_wing samolot;
        vec = samolot.collison_object();
        configure();
    }

    void Planner::drawSphere(Vec3f vector, float radius)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vector[0];
        marker.pose.position.y = vector[1];
        marker.pose.position.z = vector[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // SCALE - WE USE IT WHEN WE CHANGE RESOLUTION
        marker.scale.x = radius *2;
        marker.scale.y = radius *2;
        marker.scale.z = radius *2;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0);
        visPub.publish( marker );
    }

    void Planner::drawCUBE(Vec3f vector , int id , int c_color, float x, float y, float z)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";  //dzialamy w ukladzie map
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vector[0];
        marker.pose.position.y = vector[1];
        marker.pose.position.z = vector[2];
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;//poseQ[1];
        marker.pose.orientation.z = 0;//poseQ[2];
        marker.pose.orientation.w = 1;//poseQ[3];

        // SCALE - WE USE IT WHEN WE CHANGE RESOLUTION
        marker.scale.x = x;
        marker.scale.y = y;
        marker.scale.z = z;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;

        if(c_color == 1)
        {
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            marker.color.g = 0.0;
        }

        else if(c_color == 2)
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
        }

        else
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 1.0;
        }

        marker.lifetime = ros::Duration(0);
        visCubePub.publish(marker) ;
    }

    void Planner::drawCYLINDER(Vec3f vector ,Vec3f rot, int id , int c_color,float radius, float lz)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";  //dzialamy w ukladzie map
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vector[0];
        marker.pose.position.y = vector[1];
        marker.pose.position.z = vector[2];
        marker.pose.orientation.x = rot[0];
        marker.pose.orientation.y = rot[1];//poseQ[1];
        marker.pose.orientation.z = rot[2];//poseQ[2];
        marker.pose.orientation.w = 1;//poseQ[3];

        // SCALE - WE USE IT WHEN WE CHANGE RESOLUTION
        marker.scale.x = radius*2;
        marker.scale.y = radius*2;
        marker.scale.z = lz;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        if(c_color == 1)
        {
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            marker.color.g = 0.0;
        }
        else if(c_color == 2)
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 1.0;
        }
        marker.lifetime = ros::Duration(0);
        visCylinderPub.publish(marker) ;
    }

    void Planner::setTransform(std::vector<CollisionObject>& vector, const ob::RealVectorStateSpace::StateType *pos, const ob::SO3StateSpace::StateType *rot){

        for(auto & i : vector){
            fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
            fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
            i.setTransform(rotation, translation);
        }
    }

    bool Planner::isStateValid(const ob::State *state)
    {
        // cast the abstract state type to the type we expect
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        fcl::CollisionObject treeObj((tree_obj));

        setTransform(vec,pos,rot);

        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;

        for(auto & i : vec)
        {
            fcl::collide(&i, &treeObj, requestType, collisionResult);

            if(collisionResult.isCollision())
            {
                return !collisionResult.isCollision();
            }
        }

        return true;
    }

    nav_msgs::Path Planner::extractPath(ob::ProblemDefinition* pd){
        std::cout << "Found solution:" << std::endl;
        ob::PathPtr path = pd->getSolutionPath();
        auto* pth = pd->getSolutionPath()->as<og::PathGeometric>();
        pth->printAsMatrix(std::cout);
        // print the path to screen

        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";

        for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
        {
            const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            geometry_msgs::PoseStamped pose;

            pose.pose.position.x = pos->values[0];
            pose.pose.position.y = pos->values[1];
            pose.pose.position.z = pos->values[2];

            pose.pose.orientation.x = rot->x;
            pose.pose.orientation.y = rot->y;
            pose.pose.orientation.z = rot->z;
            pose.pose.orientation.w = rot->w;

            msg.poses.push_back(pose);

        }
        return msg;
    }

    nav_msgs::Path Planner::planPath(){

        si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));
        // set State Validity Checking Resolution (avoid going through the walls)
        si->setStateValidityCheckingResolution(0.001);

//        pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));

        // create planner
        auto planner(std::make_shared<og::FMT>(si));
        // configure the planner

        //RRTstar
//        planner->setRange(0.1);// max step length

        //SST
//        planner->setSelectionRadius(1.0);

        //FMT:
//        planner->setNumSamples(100000);
//        planner->setNearestK(false);
//        planner->setRadiusMultiplier(0.9);

        planner->setProblemDefinition(pdef);
        planner->setup();

        si->printSettings(std::cout);

        pdef->print(std::cout);

        // solve motion planning problem
        ob::PlannerStatus solved = planner->ob::Planner::solve(30.0);

        nav_msgs::Path plannedPath;

        if (solved) {
            // if cussess
            // get the planned path
            plannedPath=extractPath(pdef.get());
        }

        else{ std::cout << "No solution found" << std::endl; }

        return plannedPath;
    }
    void Planner::configure(){

//        const std::string filename = "/home/rafal/miapr_ws/src/mapr/clouds/fr_078_tidyup.bt";
        const std::string filename = "/home/rafal/miapr_ws/src/mapr/clouds/corridor.bt";

        auto* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(filename)));
        tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);

        space = ob::StateSpacePtr(new ob::SE3StateSpace());


        // create a start state
        ob::ScopedState<ob::SE3StateSpace> start(space);

        // create a goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space);

        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);

    //dla corridor:
        bounds.setLow(0,-5);
        bounds.setHigh(0,20);
        bounds.setLow(1,-5);
        bounds.setHigh(1,5);
        bounds.setLow(2,0);
        bounds.setHigh(2,3);

    //dla fr_078_tidyup.bt
//        bounds.setLow(0,-10);
//        bounds.setHigh(0,10);
//        bounds.setLow(1,-10);
//        bounds.setHigh(1,10);
//        bounds.setLow(2,0);
//        bounds.setHigh(2,2.5);


        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // construct an instance of  space information from this state space
        si = std::make_shared<ob::SpaceInformation>(space);

        //dla fr_078_tidyup.bt
//        start->setXYZ(0,-4.0,0.101);
//        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
//
//        goal->setXYZ(-8,2.5,0.101);
//        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        // dla corridor:
        start->setXYZ(1,0.0,0.101);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

        goal->setXYZ(17,3.0,0.101);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();


        si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));

        pdef = std::make_shared<ob::ProblemDefinition>(si);

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // set Optimizattion objective
        pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));

        std::cout << "Initialized: " << std::endl;

    }

    ob::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si1)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si1));
        obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
        return obj;
    }
}