#include <gflags/gflags.h>
#include "slam.h"

DEFINE_string(configFile, "../config/default.yaml", "config file path");

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    SLAM::Ptr slam(new SLAM(FLAGS_configFile));
    assert(slam->Initialize() == true);
    slam->Run();
    return 0;
}