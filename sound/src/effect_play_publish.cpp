#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>

ros::Publisher effect_pub;
// 音源ファイルのリスト
std::vector<std::string> effect_files = {
    "/home/deguchi-a/demo/src/sound/src/sound1.mp3",
    "/home/deguchi-a/demo/src/sound/src/sound2.mp3",
    "/home/deguchi-a/demo/src/sound/src/sound3.mp3",
    "/home/deguchi-a/demo/src/sound/src/sound4.mp3",
    "/home/deguchi-a/demo/src/sound/src/sound5.mp3"
};

// IDに基づいて音源ファイルをパブリッシュする
void effectIdCallback(const std_msgs::Int32::ConstPtr& msg) {
    int effect_id = msg->data;
    std_msgs::String effect_path_msg;
        if(effect_id >= 0 && effect_id < effect_files.size()){
            effect_path_msg.data = effect_files[effect_id];
            effect_pub.publish(effect_path_msg);
            ROS_INFO("Published effect path : %s",effect_path_msg.data.c_str());
        }else{
            ROS_WARN("NO Received effect ID : %d",effect_id);
        }

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "effect_publish_node");
    ros::NodeHandle nh;
    effect_pub = nh.advertise<std_msgs::String>("/effect_path", 10);
    ros::Subscriber effect_id_sub = nh.subscribe("/effect_id_request", 10, effectIdCallback);
    ros::spin();
    return 0;
}
