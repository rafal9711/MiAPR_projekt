#include "../include/mapr/fixed_wing.h"
#include "../include/ompl_example_2d/Check_Validator.h"

std::vector<CollisionObject> fixed_wing::collison_object(){
    std::vector<CollisionObject> vec;
    Vec3f trans1(1.1,0.,0.051);
    Vec3f trans2(1.,0.,0.051);
    fcl::Matrix3f R((0.0),(1.0),(0.0));

    //Create SPHERE
    float radius = 0.05;
    std::shared_ptr<Sphere> Shpere(new Sphere(radius));
    CollisionObject so0(Shpere);
    so0.setTranslation(trans1);
    vec.push_back(so0);

    //Create CUBE
    float x,y,z;
    x = 0.3;
    y=0.5;
    z=0.05;
    std::shared_ptr<Box> Box(new fcl::Box(x,y,z));
    CollisionObject bo0(Box);
    bo0.setTranslation(trans2);
    vec.push_back(bo0);

    //Create CYLINDER
    radius = 0.05;
    float lz=0.2;
    std::shared_ptr<Cylinder> Cylinder(new fcl::Cylinder(radius,lz));
    CollisionObject co0(Cylinder);
    co0.setTranslation(trans2);
    co0.setRotation(R);

    vec.push_back(co0);
    return vec;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "figure_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
//    ros::Subscriber map_sub = n.subscribe("/octomap_binary", 10, octomapCallback);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("planned_path", 1000);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    check_validator::Planner planner_(n);
    Vec3f trans1(1.1,0.,0.051);
    Vec3f trans2(1.,0.,0.051);
    Vec3f rot1(0.,1.,0.);

    float radiusS = 0.05;
    float x,y,z;
    x = 0.1;
    y = 0.45;
    z = 0.05;
    float radiusC = 0.05;
    float lz=0.2;

    while (ros::ok()){

        planner_.drawSphere(trans1,radiusS);
        planner_.drawCUBE(trans2, 1000, 3,x,y,z);
        planner_.drawCYLINDER(trans2, rot1,100, 3,radiusC,lz);

        nav_msgs::Path plannedPath;
        plannedPath = planner_.planPath();

        // publish the planned path
        path_pub.publish(plannedPath);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
//        if (std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() >=10.0){
//            cout << "STOP"<< std::endl;
//        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}