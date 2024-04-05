#include "config.h"

bool Config::setParameterFile(const std::string& filename) {
    if (_config == nullptr)
        _config = std::shared_ptr<Config>(new Config);
    _config->_file = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);

    if (_config->_file.isOpened() == false) {
        LOG(ERROR) << "Parameter file " << filename << " not found";
        _config->_file.release();
        return false;
    }
    return true;
}

Config::~Config() {
    if (_file.isOpened())
        _file.release();
}

std::shared_ptr<Config> Config::_config = nullptr;