#ifndef PACKAGESECTION_H
#define PACKAGESECTION_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "ParameterSection.hpp"

class PackageSection
{
public:
    PackageSection(const std::string& name, const YAML::Node& node);
    std::string getName() const;
    std::vector<std::shared_ptr<ParameterSection>> getParameters();
    std::string toString() const;
private:
    std::string name;
    std::vector<std::shared_ptr<ParameterSection>> parameters;
};

#endif