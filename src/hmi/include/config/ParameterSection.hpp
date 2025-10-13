#ifndef PARAMETERSECTION_H
#define PARAMETERSECTION_H

#include <string>
#include <variant>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

enum ParameterSectionType
{
    BOOL,
    DOUBLE,
    INT,
    UNSIGNED_INT,
};

/**
 * @class ParameterSection
 * @brief Represents a parameter for a package with various types and constraints.
 *
 * This class encapsulates a parameter with a name, type, value, and optional slider constraints.
 * It provides methods to retrieve the parameter's properties and convert its value to different types.
 */
class ParameterSection
{
public:
    /**
     * @brief Destructor for the ParameterSection class.
     */
    virtual ~ParameterSection();
    /**
     * @brief Constructor for the ParameterSection class.
     *
     * @param parameter The YAML node containing the parameter's properties.
     */
    explicit ParameterSection(const YAML::detail::iterator_value& parameter);
    /**
     * @brief Gets the minimum slider value.
     *
     * @return The minimum slider value.
     */
    double getMin() const;
    /**
     * @brief Gets the maximum slider value.
     *
     * @return The maximum slider value.
     */
    double getMax() const;
    /**
     * @brief Gets the name of the parameter.
     *
     * @return The name of the parameter.
     */
    std::string getName() const;
    /**
     * @brief Gets the type of the parameter.
     *
     * @return The type of the parameter as a ParameterSectionType.
     */
    ParameterSectionType getType() const;

    bool getAsBool() const;
    double getAsDouble() const;
    int getAsInt() const;
    unsigned int getAsUnsignedInt() const;

    /**
     * @brief Gets the step size of the parameter.
     *
     * @return The step size of the parameter.
     */
    double getStep() const;
    /**
     * @brief Sets the current value of the parameter.
     *
     * @param value The new value to set.
     */
    void setValue(const std::variant<bool, double, int, unsigned int> value);

    /**
     * @brief Converts a string representation of a type to a ParameterSectionType.
     *
     * @param type The type as a string.
     * @return The corresponding ParameterSectionType.
     */
    ParameterSectionType stringToType(const std::string& type) const;

    std::string getValueAsString() const;

    /**
     * @brief Converts the parameter to a string representation.
     *
     * @return The parameter as a string.
     */
    std::string toString() const;
private:
    std::string name; /* The name of the parameter. */
    YAML::Node node /* The YAML node containing the parameter's properties. */;
};

#endif