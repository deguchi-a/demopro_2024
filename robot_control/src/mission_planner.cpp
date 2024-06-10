#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <std_msgs/Bool.h>

bool movement_done = false;

void movementEndCallback(const std_msgs::Bool::ConstPtr& msg) {
    movement_done = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_planner");
    ros::NodeHandle nh;
    ros::Publisher movement_pub = nh.advertise<geometry_msgs::Vector3>("/movement_commands", 10);
    ros::Subscriber end_sub = nh.subscribe("/movement_end", 10, movementEndCallback);
    ros::Rate loop_rate(10);

    std::vector<geometry_msgs::Vector3> movements;
    
    //その場でn回旋回
    geometry_msgs::Vector3 move1;
    move1.x = 0.0;
    move1.y = 15.0;
    move1.z = 0.0;
    movements.push_back(move1);

    //0.3m直進
    geometry_msgs::Vector3 move2;
    move2.x = 1.0;
    move2.y = 0.3;
    move2.z = 0.0;
    movements.push_back(move2);

    //半径0.3mをn回転
    geometry_msgs::Vector3 move3;
    move3.x = 0.0;
    move3.y = 1.0;
    move3.z = 0.3;
    movements.push_back(move3);

    //0.7m直進
    geometry_msgs::Vector3 move4;
    move4.x = 1.0;
    move4.y = 0.7;
    move4.z = 0.0;
    movements.push_back(move4);

    //半径1.0mをn回転
    geometry_msgs::Vector3 move5;
    move5.x = 0.0;
    move5.y = 1.0;
    move5.z = 1.0;
    movements.push_back(move5);

    ros::Duration(1.0).sleep();

    for (const auto& movement : movements) {
        movement_done = false;
        movement_pub.publish(movement);
        while (ros::ok() && !movement_done) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
