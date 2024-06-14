#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sound_play/sound_play.h>
#include <time.h>

ros::Rate* loop_rate_ptr = nullptr;

// 指定したIDの音源ファイルを受け取り再生する(5秒間再生)
void effectPathCallback(const std_msgs::String::ConstPtr& msg) {
    ros::Time start = ros::Time::now();
    std::string effect_path = msg->data;
    sound_play::SoundClient sc;
    while(ros::ok()){
        ros::Duration(1.0).sleep();
        ros::Time now = ros::Time::now();
        sc.playWave(effect_path);
        ROS_INFO("Playing effect : %s",effect_path.c_str());
        if(now - start >= ros::Duration(5.0)){
            break;
        }
        ros::spinOnce();
        loop_rate_ptr->sleep();
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "effect_subscribe_node");
    ros::NodeHandle nh;

    // 効果音のパスを受け取るためのサブスクライバー
    ros::Subscriber effect_path_sub = nh.subscribe("/effect_path", 10, effectPathCallback);

    // 指定する効果音IDをリクエストするためのパブリッシャー
    ros::Publisher effect_id_pub = nh.advertise<std_msgs::Int32>("/effect_id_request", 10);
    ros::Rate loop_rate(10);
    loop_rate_ptr = &loop_rate;

    std_msgs::Int32 effect_id_msg;
    effect_id_msg.data = 2; //流したい音源の番号を指定
    ros::Duration(1.0).sleep();
    effect_id_pub.publish(effect_id_msg);
    ROS_INFO("Requested effect ID: %d", effect_id_msg.data);
    ros::spin();

    return 0;
}
