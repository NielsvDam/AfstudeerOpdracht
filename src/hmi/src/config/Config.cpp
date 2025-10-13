#include "config/Config.hpp"
#include "config/PackageSection.hpp"
#include <fstream>

Config::~Config() {}

Config::Config() : config(YAML::LoadFile("settings.yaml"))
{
    for (const auto& component : config["package_settings"])
    {
        std::string componentName = component.first.as<std::string>();
        YAML::Node componentNode = component.second;
        PackageSection packageSection(componentName, componentNode);
        packageSections.push_back(packageSection);
    }
}


std::vector<PackageSection>& Config::getPackageSections()
{
    return packageSections;
}

void Config::save()
{
    YAML::Emitter out;
    out << config;

    std::ofstream fout("settings.yaml");
    if (!fout.is_open())
    {
        throw std::runtime_error("Unable to open settings.yaml for writing");
    }

    fout << out.c_str();
    fout.close();
}

std::string Config::toString() const
{
    std::string str = "package_settings:\n";
    for (auto it = packageSections.begin(); it != packageSections.end(); ++it)
    {
        std::string packageStr = it->toString();
        std::istringstream iss(packageStr);
        std::string line;
        while (std::getline(iss, line))
        {
            str += "  " + line;
            // Only add a newline if we're not at the end of the last parameter
            if (it != packageSections.end() - 1 || !iss.eof())
            {
                str += "\n";
            }
        }
    }
    return str;
}
