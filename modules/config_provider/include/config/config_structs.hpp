#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

// Create Struct which extends the _Config struct_
// Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct DemoConfig : public Config {
    DemoConfig() : Config("test_file.toml") {}

    // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method
    // Arguments are:
    //      1. The _toml_key_: This key has to be unique for every parameter in this Config
    //      2. The _description_: A description for this parameter
    //      3. The group this parameter will be stored in (put an empty string for the global group)
    //      4. The default value of this parameter (used when no value was found in the config)
    //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
    //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
    BoolParam bool_param = createBoolParam("toml_key", "A bool param", "var_group", false);

    static constexpr int MAX_VAL = 10;
    IntParam int_param = createIntParam("key_for_int_var", "Descr. for int", "var_group", 3, 0, MAX_VAL);

    static constexpr double PI = 3.1415;
    DoubleParam pi = createDoubleParam("pi_key", "Descr. for double", "another_group", PI);

    StringParam sp = createStringParam("strParam", "A String Param", "", "string val");

    StringParam strin_param_locked =
        createStringParam("strParam2", "A String Param", "", "default string value", false);
};

}  // namespace luhsoccer::config_provider