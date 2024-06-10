#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
    //初期化関連
    ros::init(argc, argv, "music_play_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //sound_playノードが起動するまでの待機時間
    ros::Duration(1.0).sleep();

    //sound_playノードとの通信
    sound_play::SoundClient sc;

    // sound_playノードが初期化されるまでの待機時間
    ros::Duration(1.0).sleep();


    std::string music_play;
    pnh.getParam("music_file_path",music_play);
    
    // if (!music_play.empty() && linear_x > 0.0){
    //     sc.playWave(music_play);
    // }
    // else if(linear_x == 0.0){
    //     sc.stopWave(music_play);
    // }

    ros::spin();
    return 0;
}
