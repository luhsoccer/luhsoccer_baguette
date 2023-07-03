
#pragma once

#include <iostream>

namespace luhsoccer::config_provider::datatypes {

/**
 * @brief Specifies whether debug prints should be active
 *
 */
constexpr bool DEBUG_PRINTS = false;

/**
 * @brief A Enum to tell the config what sort of behaviour it should have during **development mode**
 * SHARED: Config should be saved in the cmake_rc folder
 * LOCAL: Config (changes) should be saved in the project folder under runtime/configs (this folder will be gitignored)
 */
enum class ConfigType { SHARED, LOCAL };

/**
 * @brief A Enum used to store all possible Parameter types
 *
 */
enum class Type {
    INT,
    DOUBLE,
    BOOL,
    STRING,

};

}  // namespace luhsoccer::config_provider::datatypes
