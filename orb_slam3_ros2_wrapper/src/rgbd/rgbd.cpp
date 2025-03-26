#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"


#include <signal.h>
#include <execinfo.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

// Maximum number of stack frames to capture
#define MAX_STACK_FRAMES 64

// Signal handler for segmentation fault
void segfaultHandler(int sig) {
    void *trace[MAX_STACK_FRAMES];
    int trace_size = backtrace(trace, MAX_STACK_FRAMES);

    fprintf(stderr, "Error: signal %d:\n", sig);
    // Print the backtrace to stderr
    backtrace_symbols_fd(trace, trace_size, STDERR_FILENO);
    // Optionally flush stderr and exit
    _exit(1);
}



int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // Register the signal handler for SIGSEGV
    signal(SIGSEGV, segfaultHandler);
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ORB_SLAM3_Wrapper::RgbdSlamNode>(argv[1], argv[2], ORB_SLAM3::System::STEREO);
    std::cout << "============================ " << std::endl;

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();


    std::cerr << "\nExited" << std::endl;

    return 0;
}
