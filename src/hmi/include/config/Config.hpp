#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <yaml-cpp/yaml.h>

#include "PackageSection.hpp"

class Config
{
public:
    virtual ~Config();
    Config();
    std::vector<PackageSection>& getPackageSections();
    void save();
    std::string toString() const;
private:
    void readConfig();
    YAML::Node config;
    std::vector<PackageSection> packageSections;
};

#endif