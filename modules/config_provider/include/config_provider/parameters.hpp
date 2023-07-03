
#pragma once

#include <iostream>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <toml++/toml.h>

#include "basic_container.hpp"
#include "logger/logger.hpp"
#include "datatypes.hpp"
#include "utility.hpp"

namespace luhsoccer::config_provider {

/**
 * @brief A base class for all Parameter-Classes
 */
class Param {
   public:
    Param(std::string key, std::string description, std::string group, datatypes::Type datatype, toml::table& tbl,
          toml::table& diff_tbl, std::mutex& table_mtx)
        : key(std::move(key)),
          description(std::move(description)),
          group(std::move(group)),
          datatype(datatype),
          toml_table_mutex(table_mtx),
          tbl(tbl),
          diff_tbl(diff_tbl) {}

    template <typename T>
    void saveToTable(const T& val, bool value_changed) {
        std::lock_guard<std::mutex> lock(this->toml_table_mutex);

        if (value_changed) {
            util::adjustTable(&this->diff_tbl, this->key, this->group, val);
        } else {
            util::removeElement(this->diff_tbl, this->key, this->group);
        }
    }

    /**
     * @brief Resets the value of this parameter back to the default value
     * Additionally removes the entry from the diff-table
     *
     */
    virtual void resetValue() = 0;

    [[nodiscard]] datatypes::Type getType() const { return this->datatype; }
    [[nodiscard]] const std::string& getKey() const { return this->key; }
    [[nodiscard]] const std::string& getDescription() const { return this->description; }
    [[nodiscard]] const std::string& getGroupName() const { return this->group; }

    Param(Param&&) noexcept = delete;
    Param(const Param&) = delete;
    virtual ~Param() = default;
    Param& operator=(Param&&) = delete;
    Param& operator=(const Param&) = delete;

   protected:
    template <typename T>
    std::pair<T, T> loadValue(const T& current_value, bool val_has_changed) {
        std::lock_guard<std::mutex> lock(this->toml_table_mutex);

        if (!val_has_changed && initialized) util::removeElement(this->diff_tbl, this->key, this->group);
        auto fallback_value = util::valueOr<T>(this->tbl, this->key, this->group, current_value);

        // check if a unchanged value is in the default table. If this is the case we want to remove it
        if (!initialized && !util::notEqual(current_value, fallback_value)) {
            util::removeElement(this->tbl, this->key, this->group);  // TO TEST
        }

        auto new_value = util::valueOr<T>(this->diff_tbl, this->key, this->group, fallback_value);
        return {new_value, fallback_value};
    }

   protected:
    /**
     * @brief Saves the current value into the toml::table. Doesnt update the file
     */
    virtual void updateTable() = 0;

    /**
     * @brief Loads the value at position <key> from the stored toml::table
     * On the first call it also updates the default value to that of the cmake_rc-config
     */
    virtual void load() = 0;

   protected:
    std::string key;
    std::string description;
    std::string group;

    datatypes::Type datatype;

    std::mutex& toml_table_mutex;
    toml::table& tbl;
    toml::table& diff_tbl;

    bool initialized = false;

    friend class Config;
};

/**
 * @brief A Class used to store Boolean Parameters
 * Provides a load and safe function used to interact with toml files
 */
class BoolParamClass : public Param, public ValueStorage<bool> {
   public:
    /**
     * @brief Construct a new Bool Param object
     *
     * @param tbl A reference to the toml::table in which the variable is
     * @param key The key of the variable (map & toml key)
     * @param description The description of the Variable
     * @param value The actual value of the Variable/Parameter
     * @param writable Whether the value can be changed during runtime (true -> Value can be changed, false -> Value
     * cant change, not even through config loads)
     */
    BoolParamClass(toml::table& tbl, toml::table& diff_tbl, std::string key, std::string description, std::string group,
                   bool value, bool writable, std::mutex& table_mtx)
        : Param(std::move(key), std::move(description), std::move(group), datatypes::Type::BOOL, tbl, diff_tbl,
                table_mtx),
          ValueStorage<bool>(value, writable) {}

    void resetValue() override { ValueStorage<bool>::resetValue(); }

    BoolParamClass& operator=(bool new_val) {
        this->set(new_val);
        return *this;
    }

   protected:
    void load() override {
        auto [new_value, fallback_value] = this->loadValue<bool>(this->val(), this->hasValueChanged());
        this->forceSet(new_value);
        if (!this->initialized) {
            this->initialized = true;
            this->setDefaultValue(fallback_value);
        }
    }

    void updateTable() override { this->saveToTable<bool>(this->val(), this->hasValueChanged()); }

    friend class Config;
};

/**
 * @brief A Class used to store Integer Parameters
 * Provides a load and safe function used to interact with toml files
 */
class IntParamClass : public Param, public MinMaxValStorage<int> {
   public:
    /**
     * @brief Construct a new Int Param object
     *
     * @param tbl A reference to the toml::table in which the variable is
     * @param key The key of the variable (map & toml key)
     * @param description The description of the Variable
     * @param value The actual value of the Variable/Parameter
     * @param min The minimum possible value of the parameter (not enforced)
     * @param max The maximum possible value of the parameter (not enforced)
     * @param writable Whether the value can be changed during runtime (true -> Value can be changed, false -> Value
     * cant change, not even through config loads)
     */
    IntParamClass(toml::table& tbl, toml::table& diff_tbl, std::string key, std::string description, std::string group,
                  int value, int min, int max, bool writable, std::mutex& table_mtx)
        : Param(std::move(key), std::move(description), std::move(group), datatypes::Type::INT, tbl, diff_tbl,
                table_mtx),
          MinMaxValStorage<int>(value, min, max, writable) {}

    void resetValue() override { ValueStorage<int>::resetValue(); }

    IntParamClass& operator=(int new_val) {
        this->set(new_val);
        return *this;
    }

   protected:
    void updateTable() override {
        this->saveToTable<int64_t>(static_cast<int64_t>(this->val()), this->hasValueChanged());
    }

    void load() override {
        auto [new_value, fallback_value] =
            static_cast<std::pair<int, int>>(this->loadValue<int64_t>(this->val(), this->hasValueChanged()));
        this->forceSet(new_value);
        if (!this->initialized) {
            this->initialized = true;
            this->setDefaultValue(fallback_value);
        }
    }

    friend class Config;
};

/**
 * @brief A Class used to store Double Parameters
 * Provides a load and safe function used to interact with toml files
 */
class DoubleParamClass : public Param, public MinMaxValStorage<double> {
   public:
    /**
     * @brief Construct a new Double Param object
     *
     * @param tbl A reference to the toml::table in which the variable is
     * @param key The key of the variable (map & toml key)
     * @param description The description of the Variable
     * @param value The actual value of the Variable/Parameter
     * @param min The minimum possible value of the parameter (not enforced)
     * @param max The maximum possible value of the parameter (not enforced)
     * @param writable Whether the value can be changed during runtime (true -> Value can be changed, false -> Value
     * cant change, not even through config loads)
     */
    DoubleParamClass(toml::table& tbl, toml::table& diff_tbl, std::string key, std::string description,
                     std::string group, double value, double min, double max, bool writable, std::mutex& table_mtx)
        : Param(std::move(key), std::move(description), std::move(group), datatypes::Type::DOUBLE, tbl, diff_tbl,
                table_mtx),
          MinMaxValStorage<double>(value, min, max, writable) {}

    void resetValue() override { ValueStorage<double>::resetValue(); }

    DoubleParamClass& operator=(double new_val) {
        this->set(new_val);
        return *this;
    }

   protected:
    void updateTable() override { this->saveToTable(this->val(), this->hasValueChanged()); }

    void load() override {
        auto [new_value, fallback_value] = this->loadValue<double>(this->val(), this->hasValueChanged());
        this->forceSet(new_value);
        if (!this->initialized) {
            this->initialized = true;
            this->setDefaultValue(fallback_value);
        }
    }

    friend class Config;
};

/**
 * @brief A Class used to store String Parameters
 * Provides a load and safe function used to interact with toml files
 */
class StringParamClass : public Param, public ValueStorage<std::string> {
   public:
    /**
     * @brief Construct a new String Param object
     *
     * @param tbl A reference to the toml::table in which the variable is
     * @param key The key of the variable (map & toml key)
     * @param description The description of the Variable
     * @param value The actual value of the Variable/Parameter
     * @param writable Whether the value can be changed during runtime (true -> Value can be changed, false -> Value
     * cant change, not even through config loads)
     */
    StringParamClass(toml::table& tbl, toml::table& diff_tbl, std::string key, std::string description,
                     std::string group, std::string value, bool writable, std::mutex& table_mtx)
        : Param(std::move(key), std::move(description), std::move(group), datatypes::Type::STRING, tbl, diff_tbl,
                table_mtx),
          ValueStorage<std::string>(std::move(value), writable) {}

    void resetValue() override { ValueStorage<std::string>::resetValue(); }

    StringParamClass& operator=(const std::string& new_val) {
        this->set(new_val);
        return *this;
    }

   protected:
    void updateTable() override { this->saveToTable<std::string>(this->val(), this->hasValueChanged()); }

    void load() override {
        auto [new_value, fallback_value] = this->loadValue<std::string>(this->val(), this->hasValueChanged());
        this->forceSet(new_value);
        if (!this->initialized) {
            this->initialized = true;
            this->setDefaultValue(fallback_value);
        }
    }

    friend class Config;
};

using BoolParam = BoolParamClass&;
using IntParam = IntParamClass&;
using StringParam = StringParamClass&;
using DoubleParam = DoubleParamClass&;

}  // namespace luhsoccer::config_provider
