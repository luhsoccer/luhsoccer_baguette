
#include <fstream>
#include <filesystem>

#include "common_types.hpp"

#include "logger/logger.hpp"

#include "config_provider/utility.hpp"

#include <cmrc/cmrc.hpp>
CMRC_DECLARE(config_provider);

namespace luhsoccer::config_provider::util {

bool saveFile(const std::string& path, const toml::table& tbl) {
    if (tbl.empty()) {
        if (std::filesystem::exists(path)) {
            std::error_code err_code;
            std::filesystem::remove(path, err_code);
        }
        return false;
    }

    std::ofstream file(path, std::ofstream::trunc);

    if (file.is_open()) {
        file << toml::toml_formatter{tbl, toml::format_flags::relaxed_float_precision};
        file.close();
        return true;
    }

    logger::Logger tmp_logger("config_provider");
    LOG_WARNING(tmp_logger, "Failed to open file '{}'!", path);

    return false;
}

bool parseFile(const std::string& path, toml::table& tbl, bool cmake_rc_fs, bool clear) {
    // the string containing the content of the file
    toml::parse_result result;

    if (cmake_rc_fs) {
        // CMAKE_RC FILESYSTEM //

        // Open file in internal (cmake_rc) memory
        auto fs = cmrc::config_provider::get_filesystem();

        // Check if file exists
        if (!fs.exists(path) || !fs.is_file(path)) return false;

        // open & read file
        auto file = fs.open(path);
        std::string data{file.begin(), file.end()};
        result = toml::parse(data);

    } else {
        // NORMAL FILESYSTEMM //
        // Let toml++ handle the file stuff
        result = toml::parse_file(path);
    }

    if (result.failed()) {
        logger::Logger tmp_logger("config_provider");
        // if we want to look into the cmake_rc filesystem and the file could not be parsed this is a warning (the main
        // config file could not be parsed). Otherwise the optional file could not be parsed. Then it is a Info
        if (cmake_rc_fs) {
            LOG_WARNING(tmp_logger, "Failed to read / parse toml file '{}'!", path);
        } else {
            LOG_INFO(tmp_logger, "Failed to read / parse (Optional) toml file '{}'!", path);
        }

        return false;
    }

    // If clear is enabled the table's content will be overwritten
    if (clear) {
        tbl = result.table();
        return true;
    }

    // Otherwise we have to iterate over the new content and 'insert_or_assign' the new parameters
    for (const auto& [key, value] : result.table()) {
        tbl.insert_or_assign(key, value);
    }

    return true;
}

const std::string pathLookup(const std::string& filename, const std::string_view dir_name) {
    namespace fs = std::filesystem;

    auto path = fs::current_path();

    while (true) {
        if (!path.has_filename()) {
            LOG_WARNING(logger::Logger("config_provider"), "Could not find '{}' folder!", dir_name);
            break;
        }

        if (path.filename().string() == dir_name) {
            return path.string() + "/configs/" + filename;
        }

        if (!path.has_parent_path()) {
            LOG_WARNING(logger::Logger("config_provider"), "Could not find '{}' folder!", dir_name);
            break;
        }

        path = path.parent_path();
    }

    return fs::current_path().string() + "/configs/" + filename;
}

std::string deriveConfigName(const std::string& filename) {
    // Set the name of the config equal to the filename without the dot
    const auto dot_pos = filename.find('.');
    if (dot_pos == std::string::npos) {
        return filename;
    } else {
        return filename.substr(0, dot_pos);
    }
}

bool removeElement(toml::table& tbl, const std::string& key, const std::string& group) {
    if (group.empty()) {
        bool erased = tbl.erase(key) > 0;
        tbl.prune(true);
        return erased;
    }

    const auto& sub_tbl = tbl.at_path(group);
    if (!sub_tbl.is_table()) return false;

    bool erased = sub_tbl.as_table()->erase(key);
    sub_tbl.as_table()->prune(true);
    return erased;
}

toml::table concatTables(const toml::table& tbl1, const toml::table& tbl2) {
    toml::table new_table = tbl1;

    for (const auto& [key, value] : tbl2) {
        if (value.is_table()) {
            const toml::table* inner_tbl = value.as_table();
            if (inner_tbl == nullptr) continue;

            new_table.insert(key, toml::table());
            toml::table* sub_table = new_table.at_path(key).as_table();

            for (const auto& [inner_key, inner_value] : *inner_tbl) {
                sub_table->insert_or_assign(inner_key, inner_value);
            }
        } else {
            new_table.insert_or_assign(key, value);
        }
    }

    return new_table;
}

}  // namespace luhsoccer::config_provider::util