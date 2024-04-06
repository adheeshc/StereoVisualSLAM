#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include "commonInclude.h"

class Config {
private:
    static std::shared_ptr<Config> _config;
    cv::FileStorage _file;
    Config();

public:
    ~Config();
    static bool setParameterFile(const std::string& filename);
    template <typename T>
    static T Get(const std::string& key) {
        return T(Config::_config->_file[key]);
    }
};

#endif  // CONFIG_H