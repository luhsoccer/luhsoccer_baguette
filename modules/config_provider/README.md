# ConfigProvider Tool {#config_provider}

Description: This Module loads, parses and stores data from config files and it makes it possible to set and get runtime parameters

# How to use: 

Config Files / Objects: 
-------------------------

0. <u>Includes:</u>
    * _"config_provider/config_store_main.hpp"_
    * Needed for Iterating over Configs (for luhviz): _"config_provider/datatypes.hpp"_

    <u>Get the Config Store</u>
    * ``` auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore(); ```


1. <u>Create Config Objects:</u>
    * In the file **_config_structs.hpp_** you can create structs containing your variables
        * A Variable can be one of the Following: 
            * _IntParam_
            * _BoolParam_
            * _DoubleParam_ 
            * _StringParam_ 
    * When creating a new config, you now have to specify whether the config should be a LOCAL or a SHARED config. 
        * changes to LOCAL configs will be saved in the runtime directory of the baguette-project directory
        * changes to SHARED configs will be saved in the extern/cmake_rc/configs directory, overwriting existing configs. These changes will be permanently added once the software is recompiled 
    * When you create a SHARED config you should add a .toml file to the extern/cmake_rc/configs directory **and** add it to the CmakeList in the cmake_rc directory

    **Example Code:** 

    _Create Config Struct: (in directory modules/config_provider/configs)_

        ```
        #pragma once

        #include "config_provider/parameters.hpp"
        #include "config_provider/config_base.hpp"

        namespace luhsoccer::config_provider {

        // Create Struct which extends the _Config struct_ 
        // Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
        struct DemoConfig : public Config {
            DemoConfig() : Config("test_file.toml", datatypes::ConfigType::SHARED) {}

            // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method 
            // Arguments are: 
            //      1. The _toml_key_: This key has to be unique for every parameter in this Config
            //      2. The _description_: A description for this parameter 
            //      3. The group this parameter will be stored in (put an empty string for the global group)
            //      4. The default value of this parameter (used when no value was found in the config)
            //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
            //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
            BoolParam bool_param = createBoolParam("toml_key", "A bool param", "var_group", false);


            IntParam int_param = createIntParam("key_for_int_var", "Descr. for int", "var_group", 3, 0, 10);

            DoubleParam pi = createDoubleParam("pi_key", "Descr. for double", "another_group", 3.1415);

            StringParam sp = createStringParam("strParam", "A String Param", "", "string val");

            StringParam strin_param_locked =
                createStringParam("strParam2", "A String Param", "", "default string value", false);
        };

        }  // namespace luhsoccer::config_provider
        ```

    _Add your config struct to the ConfigStore:_

        ```
        #pragma once

        #include "config_provider/config_base.hpp"

        // Add your config file header here
        #include "config/config_structs.hpp"

        namespace luhsoccer::config_provider {

        /**
        * @brief A Struct which stores all Config objects
        *        The Config Objects are stored as a variable and in a list so that you can iterate over them
        */
        struct ConfigStore : public ConfigStoreBase {
            // Add your config file here (AS A REFERENCE)
            DemoConfig& demo_config = addConfig<DemoConfig>();
        };

        }  // namespace luhsoccer::config_provider

        ```


2. <u>Use Config Objects:</u>
    * Get the ConfigStore via the method described above!
    * Acces the value of a Parameter by using it just like a normal Variable or alternatively through the _val()_ method 
    * Set the value of a Parameter via the _set()_ method or the _=_ operator 

    **Example Code:**
    ```
    #include "config_provider/config_store_main.hpp"

    ...

    // Get the ConfigStore
    auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore(); 

    // Read the Value of the variable
    std::cout << cs.demo_config.pi << std::endl;
    // or
    std::cout << cs.demo_config.pi.val() << std::endl;

    // Assign new Value
    cs.demo_config.pi = 3.1415;
    // or
    cs.demo_config.pi.set(3.1415);

    // Get the Min & Max value (Only for Ints & Doubles)
    std::cout << cs.demo_config.pi.getMin() << std::endl;
    std::cout << cs.demo_config.pi.getMax() << std::endl;

    // Optional stuff
    std::cout << cs.demo_config.pi.getDescription() << std::endl;
    std::cout << cs.demo_config.pi.getKey() << std::endl;
    std::cout << cs.demo_config.pi.getGroupName() << std::endl;
    std::cout << cs.demo_config.getConfigName() << std::endl;

    ```

3. <u>Iterate Over Config Objects:</u>
    * To iterate over all configs, use the _getConfigs()_ method of the ConfigStore class to get a std::vector containing all Config objects
    * Then use the _getParams()_ method of the config Class to get a map of all Parameters
    * You can then iterate over the map and retreive all Parameters within
    * You can use the _getType()_ method of the Parameter to determine its type and then you need to cast the parameter to its specific Parameter 
    * Then you can access the value of the parameter
    * **Note:** Configs and Parameters are stores as _std::unique_ptr_ in this case
    
    **Example code:** 
    ```
    luhsoccer::config_provider::ConfigStore cs; 

    // Iterate over all config structs
    for (auto& cfg : cs.getConfigs()) {
        // Iterate over all paramters withing the current config struct
        for (auto& i : cfg->getParams()) {

            // look if the value has changed. This can be used to see when a parameter should be resettable
            bool has_changed = i.second->hasValueChanged(); 
            
            // Reset the value of the parameter to the default value 
            i.second->resetValue(); 

            // evaluate the type of the parameter, cast it and access its value / change it
            switch (i.second->getType()) {
                case luhsoccer::config_provider::datatypes::Type::INT: {
                    auto& el = dynamic_cast<luhsoccer::config_provider::IntParam>(*i.second);
                    el = 5;
                    std::cout << el.getKey() << " " << el << std::endl;
                    break;
                }
                case luhsoccer::config_provider::datatypes::Type::DOUBLE: {
                    auto& el = dynamic_cast<luhsoccer::config_provider::DoubleParam>(*i.second);
                    std::cout << el.getKey() << " " << el << std::endl;
                    break;
                }
                case luhsoccer::config_provider::datatypes::Type::BOOL: {
                    auto& el = dynamic_cast<luhsoccer::config_provider::BoolParam>(*i.second);
                    std::cout << el.getKey() << " " << el << std::endl;
                    break;
                }
                case luhsoccer::config_provider::datatypes::Type::STRING: {
                    auto& el = dynamic_cast<luhsoccer::config_provider::StringParam>(*i.second);
                    std::cout << el.getKey() << " " << el.val() << std::endl;
                    break;
                }
            }
        }
        std::cout << std::endl;
    }


    ```


4. <u>Load and Save Config Files:</u>
    * To load the data from a .toml file into a config object you can use the _load()_ method of the config (however config files are also loaded as soon as the config is created)
    * To save the data in a config instance you can call the _save()_ method of the config object
        * Note: Parameters can only be saved to a .toml file if 
            1. There is no Variable with the same name in the .toml file -> **KEYS HAVE TO BE UNIQUE**
