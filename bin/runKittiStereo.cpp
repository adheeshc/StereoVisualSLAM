#include <gflags/gflags.h>
#include <glog/logging.h>
#include "slam.h"

DEFINE_string(configFile, "../config/default.yaml", "config file path");

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    // Initialize glog with file logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;  // Log to both file and stderr
    FLAGS_colorlogtostderr = true;  // Colored terminal output
    FLAGS_log_dir = "./logs";  // Log directory

    // Create logs directory if it doesn't exist
    system("mkdir -p ./logs");

    LOG(INFO) << "Starting Stereo Visual SLAM";

    SLAM::Ptr slam(new SLAM(FLAGS_configFile));
    assert(slam->Initialize() == true);
    slam->Run();

    google::ShutdownGoogleLogging();
    return 0;
}