#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// グローバル変数の宣言
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;
geometry_msgs::Twist twist; // 指令する速度、角速度
geometry_msgs::Twist cmd_vel;
double initial_yaw = 0.0;
double current_yaw = 0.0;
bool initial_yaw_set = false;
ros::Publisher cmd_vel_pub;
ros::Publisher yaw_pub;
ros::Publisher end_pub;
ros::Rate* loop_rate_ptr = nullptr;
double diff_yaw = 0.0;
bool movement_done = false;

// 関数プロトタイプの宣言
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);

// オドメトリのコールバック関数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_r = msg->pose.pose.orientation;
    geometry_quat_to_rpy(roll, pitch, yaw, robot_r);
    current_yaw = yaw; // current_yawを更新
}

// クォータニオンからRPYに変換する関数
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void reset_odometry() {
    ros::spinOnce();
    initial_yaw_set = false;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    initial_yaw = current_yaw;
    initial_yaw_set = true;
}

void send_end_signal() {
    std_msgs::Bool end_msg;
    end_msg.data = true;
    end_pub.publish(end_msg);
}

void rotate(double target_angle, double angular_speed) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angular_speed * (target_angle > 0 ? 1 : -1);

    double accumulated_yaw = 0.0;
    double last_yaw = current_yaw;

    while (ros::ok() && accumulated_yaw < fabs(target_angle)) {
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate_ptr->sleep();

        double delta_yaw = current_yaw - last_yaw;
        if (delta_yaw < -M_PI) {
            delta_yaw += 2 * M_PI;
        } else if (delta_yaw > M_PI) {
            delta_yaw -= 2 * M_PI;
        }
        accumulated_yaw += fabs(delta_yaw);
        last_yaw = current_yaw;
        // std::cout << "目標角度θ：" << accumulated_yaw * 180 / M_PI << std::endl;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(cmd_vel);
    ros::Duration(1.0).sleep();
    reset_odometry();
}

void move_circle(double linear_speed, int turns,double radius) {
    if (radius == 0.0) {
        double angular_speed = linear_speed;
        double target_angle = 2 * M_PI * turns;

        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = angular_speed;

        double accumulated_yaw = 0.0;
        double last_yaw = current_yaw;

        while (ros::ok() && accumulated_yaw < target_angle) {
            cmd_vel_pub.publish(cmd_vel);
            ros::spinOnce();
            loop_rate_ptr->sleep();

            double delta_yaw = current_yaw - last_yaw;
            if (delta_yaw < -M_PI) {
                delta_yaw += 2 * M_PI;
            } else if (delta_yaw > M_PI) {
                delta_yaw -= 2 * M_PI;
            }
            accumulated_yaw += fabs(delta_yaw);
            last_yaw = current_yaw;

            diff_yaw = accumulated_yaw;
            std_msgs::Float64 diff_yaw_msg;
            diff_yaw_msg.data = diff_yaw;
            yaw_pub.publish(diff_yaw_msg);
            // std::cout << "半径r:" << radius << "目標角度θ:" << diff_yaw << std::endl;
        }

    } else {
        double angular_speed_circle = linear_speed / radius;
        double target_circle_angle = 2 * M_PI * turns;

        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = angular_speed_circle;

        double accumulated_yaw = 0.0;
        double last_yaw = current_yaw;

        while (ros::ok() && accumulated_yaw < target_circle_angle) {
            cmd_vel_pub.publish(cmd_vel);
            ros::spinOnce();
            loop_rate_ptr->sleep();

            double delta_yaw = current_yaw - last_yaw;
            if (delta_yaw < -M_PI) {
                delta_yaw += 2 * M_PI;
            } else if (delta_yaw > M_PI) {
                delta_yaw -= 2 * M_PI;
            }
            accumulated_yaw += fabs(delta_yaw);
            last_yaw = current_yaw;

            diff_yaw = accumulated_yaw;
            std_msgs::Float64 diff_yaw_msg;
            diff_yaw_msg.data = diff_yaw;
            yaw_pub.publish(diff_yaw_msg);
            // std::cout << "半径r:" << radius << "目標角度θ:" << diff_yaw << std::endl;
        }
    }


    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(cmd_vel);
    ros::Duration(1.0).sleep();
    reset_odometry();
}

void move_straight(double distance, double linear_speed) {
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = 0.0;

    double initial_x = robot_x;
    double initial_y = robot_y;

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate_ptr->sleep();

        double distance_moved = sqrt(pow(robot_x - initial_x, 2) + pow(robot_y - initial_y, 2));

        if (distance_moved >= distance) {
            break;
        }

        cmd_vel_pub.publish(cmd_vel);
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(cmd_vel);

    ros::Duration(1.0).sleep();
    reset_odometry();
}

void movementCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    if (msg->x == 0.0 && msg->z == 0.0) {
        std::cout << "rotate_start" << std::endl;
        // その場旋回
        move_circle(0.5,msg->y,msg->z);
        std::cout << "rotate_end" << std::endl;
        movement_done = true;
        send_end_signal(); // 動作終了シグナルを送信
    }else if(msg->x == 0.0){
        std::cout << "circle_start" << std::endl;
        // 円弧運動
        move_circle(0.1,msg->y,msg->z);
        std::cout << "circle_end" << std::endl;
        movement_done = true;
        send_end_signal(); // 動作終了シグナルを送信
    } else if (msg->x == 1) {
        rotate(-M_PI / 2, 0.5);  // 右に90度回転
        std::cout << "straight_start" << std::endl;
        // 直進
        move_straight(msg->y, 0.1);
        std::cout << "straight_end" << std::endl;
        rotate(M_PI / 2, 0.5);  // 左に90度回転
        movement_done = true;
        send_end_signal(); // 動作終了シグナルを送信
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 10);
    yaw_pub = nh.advertise<std_msgs::Float64>("/diff_yaw", 10);
    end_pub = nh.advertise<std_msgs::Bool>("/movement_end", 10);
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 10, odomCallback);
    ros::Subscriber movement_sub = nh.subscribe("/movement_commands", 10, movementCallback);
    ros::Rate loop_rate(10);
    loop_rate_ptr = &loop_rate;

    ros::spin();

    return 0;
}
