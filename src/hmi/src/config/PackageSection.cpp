#include "config/PackageSection.hpp"

PackageSection::PackageSection(const std::string& name, const YAML::Node& node) : name(name)
{
    for (const auto& parameter : node)
    {
        parameters.emplace_back(std::make_shared<ParameterSection>(parameter));
    }
}

std::string PackageSection::getName() const
{
    return name;
}

std::vector<std::shared_ptr<ParameterSection>> PackageSection::getParameters()
{
    return parameters;
}

std::string PackageSection::toString() const
{
    std::string str = getName() + ":\n";
    for (auto it = parameters.begin(); it != parameters.end(); ++it)
    {
        std::string paramStr = (*it)->toString();
        std::istringstream iss(paramStr);
        std::string line;
        while (std::getline(iss, line))
        {
            str += "  " + line;
            // Only add a newline if we're not at the end of the last parameter
            if (it != parameters.end() - 1 || !iss.eof())
            {
                str += "\n";
            }
        }
    }
    return str;
}