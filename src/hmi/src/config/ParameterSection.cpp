#include "config/ParameterSection.hpp"
#include <stdexcept>
#include <iostream>

ParameterSection::~ParameterSection() {}

ParameterSection::ParameterSection(const YAML::detail::iterator_value& parameter)
    : name(parameter.first.as<std::string>()), node(parameter.second)
{}

double ParameterSection::getMin() const
{
    return node["slider_min"].as<double>();
}

double ParameterSection::getMax() const
{
    return node["slider_max"].as<double>();
}

std::string ParameterSection::getName() const
{
    return name;
}

bool ParameterSection::getAsBool() const
{
    return node["value"].as<bool>();
}

double ParameterSection::getAsDouble() const
{
    return node["value"].as<double>();
}

int ParameterSection::getAsInt() const
{
    return node["value"].as<int>();
}

unsigned int ParameterSection::getAsUnsignedInt() const
{
    return node["value"].as<unsigned int>();
}

double ParameterSection::getStep() const
{
    return node["step"].as<double>();
}

void ParameterSection::setValue(const std::variant<bool, double, int, unsigned int> value)
{
    switch (getType())
    {
        case BOOL:
            node["value"] = std::get<bool>(value);
            break;
        case DOUBLE:
            node["value"] = std::get<double>(value);
            break;
        case INT:
            node["value"] = std::get<int>(value);
            break;
        case UNSIGNED_INT:
            node["value"] = std::get<unsigned int>(value);
            break;
        default:
            throw std::invalid_argument("Unsupported type in variant");
    }
}

ParameterSectionType ParameterSection::getType() const
{
    std::string type = node["type"].as<std::string>();
    if (type == "BOOL")
    {
        return BOOL;
    }
    if (type == "DOUBLE")
    {
        return DOUBLE;
    }
    if (type == "INT")
    {
        return INT;
    }
    if (type == "UNSIGNED_INT")
    {
        return UNSIGNED_INT;
    }
    throw std::invalid_argument("Unknown type: " + type);
}

std::string ParameterSection::getValueAsString() const 
{
    switch (getType())
    {
        case BOOL:
            return (node["value"].as<bool>() ? "true" : "false");
        case DOUBLE:
            return std::to_string(node["value"].as<double>());
        case INT:
            return std::to_string(node["value"].as<int>());
        case UNSIGNED_INT:
            return std::to_string(node["value"].as<unsigned int>());
        default:
            throw std::invalid_argument("Unknown type");
    }
}

std::string ParameterSection::toString() const
{
    std::string str = getName() + ":\n";
    str += "  type: " + node["type"].as<std::string>() + "\n";
    str += "  value: " + getValueAsString();
    str += "\n";
    str += "  slider_min: " + std::to_string(getMin()) + "\n";
    str += "  slider_max: " + std::to_string(getMax());
    return str;
}