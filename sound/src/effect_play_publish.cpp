#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>

std_msgs::String EffectList(const std::vector<std::string>& effect_files) {
    std_msgs::String result;
    for (int i = 0; i < effect_files.size(); i++) {
        result.data += effect_files[i];
        if (i < effect_files.size() - 1) {
            result.data += ",";
        }
    }
    return result;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "effect_play_publish_node");
    ros::NodeHandle nh;
    ros::Publisher effect_list_pub = nh.advertise<std_msgs::String>("/effect_list", 10);
    // ros::Publisher effect_id_pub = nh.advertise<std_msgs::Int32>("/effect_id", 10);
    ros::Rate loop_rate(10);

    std::vector<std::string> effect_files = {
        "/home/deguchi-a/demo/src/sound/src/sound1.mp3",
        "/home/deguchi-a/demo/src/sound/src/sound2.mp3",
        "/home/deguchi-a/demo/src/sound/src/sound3.mp3",
        "/home/deguchi-a/demo/src/sound/src/sound4.mp3",
        "/home/deguchi-a/demo/src/sound/src/sound5.mp3"
    };

    std_msgs::String list_msg = EffectList(effect_files);
    // int effect_id = 0;

    while (ros::ok()) {
        // パブリッシュする音源リストメッセージを作成
        effect_list_pub.publish(list_msg);

        // パブリッシュするエフェクトIDを作成
        // std_msgs::Int32 id_msg;
        // id_msg.data = effect_id;
        // effect_id_pub.publish(id_msg);
        // ROS_INFO("Published effect_id: %d", effect_id);

        // // 次のIDに進む
        // effect_id = (effect_id + 1) % effect_files.size();

        loop_rate.sleep();
    }

    return 0;
}
