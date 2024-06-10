#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include <vector>
#include <string>
#include <sstream>
#include <time.h>

sound_play::SoundClient *sc;
std::vector<std::string> effect_files;
bool list_received = false; 

void effectListCallback(const std_msgs::String::ConstPtr& msg) {
    std::string data = msg->data;
    std::stringstream ss(data);
    std::string item;
    while (std::getline(ss, item, ',')) {
        effect_files.push_back(item);
    }
    list_received = true; 
}

void effectplay(const std_msgs::Int32& number_msg) {
    int number = number_msg.data;
    if (number >= 0 && number < effect_files.size()) {
        sc->playWave(effect_files[number-1]);
        ROS_INFO("Playing effect: %s", effect_files[number-1].c_str());
    } else {
        ROS_WARN("Invalid effect number: %d", number-1);
    }
}

// void effectCallback(const std_msgs::Int32::ConstPtr& msg) {
//     int effect_id = msg->data;
//     if (effect_id >= 0 && effect_id < effect_files.size()) {
//         sc->playWave(effect_files[effect_id]);
//         // ros::Duration(1.0).sleep();
//         ROS_INFO("Playing effect: %s", effect_files[effect_id].c_str());
//     } else {
//         ROS_WARN("Invalid effect_id: %d", effect_id);
//     }
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "effect_play_subscribe_node");
    ros::NodeHandle nh;
    sound_play::SoundClient sc_local;
    sc = &sc_local;
    ros::Duration(2.0).sleep();
    ros::Subscriber effect_list_sub = nh.subscribe("/effect_list", 10, effectListCallback);
    // ros::Subscriber effect_id_sub = nh.subscribe("/effect_id", 10, effectCallback);
    ros::Rate loop_rate(10);
    std_msgs::Int32 effect_number_msg;

    while (ros::ok()) {
        std::cout << "流したい音源の番号：";
        std::cin >> effect_number_msg.data;
        ros::Time start = ros::Time::now();
        ros::spinOnce();

        //1を選択したら1番の音源を5秒間流す(音源を何回も流すバージョン)
        if (list_received && effect_number_msg.data == 1) {
            effect_number_msg.data = 1;
            while(true){
                ros::Time now = ros::Time::now();
                effectplay(effect_number_msg);
                ros::Duration(1.0).sleep(); // 再生後に待機時間を設定
                if(now - start > ros::Duration(5.0)){
                    break;
                }
            }
        }

        //2を選択したら2番の音源を一回流す(音源を1回だけ流すバージョン)
        if (list_received && effect_number_msg.data == 2) {
            effect_number_msg.data = 2;
            effectplay(effect_number_msg);
            ros::Duration(1.0).sleep();  // 再生後に待機時間を設定
        }

        //3を選択したら3番の音源を流す
        if (list_received && effect_number_msg.data == 3) {
            effect_number_msg.data = 3;
            effectplay(effect_number_msg);
            ros::Duration(1.0).sleep();  // 再生後に待機時間を設定
        }

        //4を選択したら4番の音源を流す
        if (list_received && effect_number_msg.data == 4) {
            effect_number_msg.data = 4;
            effectplay(effect_number_msg);
            ros::Duration(1.0).sleep();  // 再生後に待機時間を設定
        }

        //5を選択したら5番の音源を流す
        if (list_received && effect_number_msg.data == 5) {
            effect_number_msg.data = 5;
            effectplay(effect_number_msg);
            ros::Duration(1.0).sleep();  // 再生後に待機時間を設定
        }
        loop_rate.sleep();
    }

    return 0;
}

