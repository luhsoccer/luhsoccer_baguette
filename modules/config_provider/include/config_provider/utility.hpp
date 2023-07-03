
#pragma once

#include <iostream>
#include <toml++/toml.h>
#include "datatypes.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::config_provider::util {

/**
 * @brief A Function used to retrieve a value from a toml::table. Returns a default value if the given key is not in the
 * table
 *
 * @tparam T The Type of the value
 * @param tbl The toml::table in which to look for the value
 * @param key The key for the valuie
 * @param default_value The default value which is returned if the value for the key was not found
 * @return T The found value or default_value
 */
template <typename T>
T valueOr(const toml::table& tbl, const std::string_view key, const std::string_view group, const T& default_value) {
    logger::Logger logger("config_provider");

    // Create an empty toml::node object
    toml::node_view<const toml::node> element;

    // Only if a group is given, look for a group
    if (group != "") {
        // get the group section of the toml::table
        element = tbl.at_path(group);

        // If the group was not found give back a default value (Dont look in the global scope to avoid
        // naming conflicts)
        if (element.type() != toml::v3::node_type::table) {
            if constexpr (datatypes::DEBUG_PRINTS) {
                LOG_DEBUG(logger, "Could not find Group '{}' for Variable '{}'! Using default Value!", group, key);
            }
            return default_value;
        }

        // Then look for the variable (in the reduced table)
        element = element.at_path(key);

    } else {
        // Then look for the variable (in the whole table)
        element = tbl.at_path(key);
    }

    // If the variable is not the specified type we have a problem -> check for types
    if (!element.is<T>()) {
        // If: The found element has a different type than the specified type, Give a specific Warning message
        // Else: there exists no element with the specified key, Give a specific Warning message
        if (element.type() != toml::v3::node_type::none) {
            LOG_WARNING(logger, "Could not load Value for key '{}' because types dont match!", key);
        } else {
            if constexpr (datatypes::DEBUG_PRINTS) {
                LOG_DEBUG(logger, "Could not find value for key '{}', using default value", key);
            }
        }

        return default_value;
    }

    // The Element was found -> Return its value or the default value if a unexpected error occured
    return element.value_or(default_value);
}

/**
 * @brief A Function used to adjust a toml::table
 *
 * @tparam T The Type of the value which is to be placed at the position at key
 * @param tbl The toml::table which contains the toml data
 * @param key The key for the value
 * @param val The value which is to be written to the position at <key>
 * @return true If the value was successfully written to the toml::table
 * @return false If the was not found in the table
 */
template <typename T>
bool adjustTable(toml::table* tbl, const std::string_view key, const std::string_view group, const T& val) {
    logger::Logger logger("config_provider");

    if (tbl == nullptr) {
        LOG_WARNING(logger, "Pointer to table is nullptr");
        return false;
    }

    // Create an empty toml::node object
    toml::node_view<toml::node> element;

    // Only if a group is given, look for a group
    if (group != "") {
        // get the group section of the toml::table
        element = tbl->at_path(group);

        // If the group was not found give back a default value (Dont look in the global scope so that there are no
        // naming collisions)
        if (element.type() != toml::v3::node_type::table) {
            // If the group was not found insert it and add its respective value
            tbl->insert(group, toml::table({{key, val}}));
            LOG_INFO(logger, "Could not find Group '{}' for Variable '{}'! Inserting...", group, key);
            return true;
        }

        // point the tbl ptr to the sub-table (the group)
        tbl = element.as_table();
    }

    // Get the element at the position <key> to check if types match
    element = tbl->at_path(key);

    // if the type of the found element matches the given type or the element doesnt exist, set/add the new value
    // and return true
    if (element.is<T>() || element.type() == toml::v3::node_type::none) {
        tbl->insert_or_assign(key, val);
        return true;
    }

    LOG_WARNING(logger, "Could not save Parameter '{}' becausae types dont match!", key);
    return false;
}

/**
 * @brief A Function which saves a toml::table to a file
 *
 * @param path The path of the File which is to be written to
 * @param tbl The table which is to be written to the file
 * @return true If the operation was successfull
 * @return false If the operation was not successfull
 */
bool saveFile(const std::string& path, const toml::table& tbl);

/**
 * @brief Parses a toml file
 *
 * @param path The path to the toml file (filename inclusive)
 * @param tbl A reference to a toml::table which will be overridden by the function
 * @param cmake_rc_fs Whether the file in question is in the cmakerc memory or in the normal filesystem
 * @param clear Discard the old content of the toml-table; On false this will only append/reassign values
 * @return true The Function parsed the file successfully and wrote the results to the toml::table
 * @return false The operation failed
 */
bool parseFile(const std::string& path, toml::table& tbl, bool cmake_rc_fs, bool clear = true);

/**
 * @brief Goes up the directory tree until it finds the directory with the name specified in 'dir_name'
 *
 * @param filename The name of the config file
 * @param dir_name The name of the directory name
 * @return const std::string&
 */
const std::string pathLookup(const std::string& filename, const std::string_view dir_name);

/**
 * @brief Derives the name of a config based on the given config name and the filename
 *
 * @param filename The name of the .toml file, the config is saved in
 * @return std::string The computed name of the config
 */
std::string deriveConfigName(const std::string& filename);

/**
 * @brief Removes the element with key 'key' from the given toml::table
 *
 * @param tbl The table to be altered
 * @param key The key of the element
 * @param group The group of the element
 * @return true The value was removed successfully
 * @return false The value could not be removed
 */
bool removeElement(toml::table& tbl, const std::string& key, const std::string& group);

/**
 * @brief Used to check if two values are unequal using the '!=' operator
 *
 * @tparam T The type of the parameters
 * @param lhs The left operator
 * @param rhs The right operator
 * @return true The arguments are unequal
 * @return false The arguments are equal
 */
template <typename T>
bool notEqual(const T& lhs, const T& rhs) {
    return lhs != rhs;
}

/**
 * @brief Checks whether two double values are not equal with a margin
 *
 * @tparam double Specific overload for doubles
 * @param lhs The left operator
 * @param rhs The right operator
 * @return true The two doubles are roughly not equal
 * @return false The two doubles are roughly equal
 */
template <>
inline bool notEqual(const double& lhs, const double& rhs) {
    constexpr double MARGIN = 0.0001;
    return (lhs < rhs - MARGIN) || (lhs > rhs + MARGIN);
}

/**
 * @brief Concatonates two toml tables
 *
 * @param tbl1 The first table (Will be the base table)
 * @param tbl2 The second table (will be the value which overrides the values from table1)
 * @return toml::table The resulting concatonated table
 */
toml::table concatTables(const toml::table& tbl1, const toml::table& tbl2);

}  // namespace luhsoccer::config_provider::util
