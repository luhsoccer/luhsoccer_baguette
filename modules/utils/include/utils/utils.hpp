#pragma once

// Here is place for your own code

#include "core/common_types.hpp"
#include <filesystem>
#include <chrono>

namespace luhsoccer {

inline double cropAngle(double angle) {
    if (angle > L_PI) {
        angle -= 2 * L_PI;
    }
    if (angle < -L_PI) {
        angle += 2 * L_PI;
    }
    return angle;
}

std::filesystem::path getBaguetteDirectory();

enum class MessageBoxIcon { INFO_ICON, WARNING_ICON, ERROR_ICON, QUESTION_ICON };

void notifySystem(const std::string_view& content, MessageBoxIcon icon);

void highPrecisionSleep(unsigned long nanos);

template <class _Rep, class _Period>
void highPrecisionSleep(const std::chrono::duration<_Rep, _Period>& time) {
    auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(time);
    highPrecisionSleep(dur.count());
}

enum class FileOptions : uint8_t {
    NONE = 0,
    // For file open, allow multiselect.
    MULTISELECT = 0x1,
    // For file save, force overwrite and disable the confirmation dialog.
    FORCE_OVERWRITE = 0x2,
    // For file open or save, force path to be the provided argument instead
    // of the last opened directory, which is the Microsoft-recommended, user-
    // friendly behaviour.
    FORCE_PATH = 0x4,
};

inline FileOptions operator|(FileOptions a, FileOptions b) { return FileOptions(uint8_t(a) | uint8_t(b)); }
inline bool operator&(FileOptions a, FileOptions b) { return bool(uint8_t(a) & uint8_t(b)); }

std::vector<std::string> openFile(const std::string& title, const std::string& initial_path,
                                  std::vector<std::string> filters = {"All Files", "*"},
                                  FileOptions options = FileOptions::NONE);

std::string saveFile(const std::string& title, const std::string& initial_path,
                     std::vector<std::string> filters = {"All Files", "*"}, FileOptions options = FileOptions::NONE);

// std::filesystem::path selectFolder(std::string_view title, std::string_view defaultPath = "");

}  // namespace luhsoccer