
#include <unistd.h>
#include <csignal>

#include "custom_logs.hpp"
#include "laser_mapping.h"

DEFINE_bool(need_logs, true, "save logs for check");
void SigHandle(int sig) {
    lio_lite::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    init_log();
    if(FLAGS_need_logs){
        save_log("local");
    }
    ros::init(argc, argv, "lio_lite");
    ros::NodeHandle nh;

    LOG(INFO) << "\033[1;32m run_mapping_online \033[0m";

    auto laser_mapping = std::make_shared<lio_lite::LaserMapping>();
    laser_mapping->InitROS(nh);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    // online, almost same with offline, just receive the messages from ros
    while (ros::ok()) {
        if (lio_lite::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();
        laser_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    lio_lite::Timer::PrintAll();

    return 0;
}
