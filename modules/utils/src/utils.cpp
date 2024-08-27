// Here is place for your own code
#include "utils/utils.hpp"
#include <filesystem>
#include "logger/logger.hpp"
#include "portable-file-dialogs.h"

#ifdef __unix__
#ifndef BAGUETTE_LOCAL_MODE
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

// Hide the getHomeDirectory function to prevent outside use
static std::filesystem::path getHomeDirectory() {
    struct passwd* pw = getpwuid(getuid());

    return {pw->pw_dir};
}

#endif
#elif defined(_WIN32) || defined(WIN32)
#ifndef BAGUETTE_LOCAL_MODE
#define NOMINMAX
#include <Shlobj.h>

static std::filesystem::path getHomeDirectory() {
    WCHAR path[MAX_PATH];
    if (SUCCEEDED(SHGetFolderPathW(NULL, CSIDL_PROFILE, NULL, 0, path))) {
        return {path};
    } else {
        return {};
    }
}

#endif
#endif

namespace luhsoccer {

#if defined(_WIN32) || defined(WIN32)
void highPrecisionSleep(unsigned long nanos) {
    auto handle = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    if (handle == 0) {
        // If we failed to create a handle just use the default sleep algorithm
        std::this_thread::sleep_for(std::chrono::nanoseconds(nanos));
        return;
    }

    LARGE_INTEGER due_time;
    std::uint64_t duration = static_cast<std::uint64_t>(nanos);
    due_time.QuadPart = ~(duration / 100);  // Windows can sleep in 100ns intervalls
    SetWaitableTimer(handle, &due_time, 0, nullptr, nullptr, false);

    WaitForSingleObject(handle, INFINITE);

    CloseHandle(handle);
}
#else
// As fallback we the the default sleep routine
void highPrecisionSleep(unsigned long nanos) { std::this_thread::sleep_for(std::chrono::nanoseconds(nanos)); }
#endif

std::filesystem::path getBaguetteDirectory() {
#ifdef BAGUETTE_LOCAL_MODE
    std::filesystem::path home_path = {BAGUETTE_LOCAL_MODE};
    home_path /= "runtime";
#else
    auto home_path = getHomeDirectory();

    // go into the hidden folder '.luhsoccer_baguette'
    home_path /= ".luhsoccer_baguette";
#endif

    // If the folder does not exist, create it
    if (!std::filesystem::is_directory(home_path)) {
        std::filesystem::create_directory(home_path);
    }

    return home_path;
}

void showSystemNotification(std::string_view content) {}

void showSystemMessageBox(const std::string& content, MessageBoxIcon icon) {
    auto icon_enum = pfd::icon::info;

    switch (icon) {
        case MessageBoxIcon::INFO_ICON:
            icon_enum = pfd::icon::info;
            break;
        case MessageBoxIcon::WARNING_ICON:
            icon_enum = pfd::icon::warning;
            break;
        case MessageBoxIcon::ERROR_ICON:
            icon_enum = pfd::icon::error;
            break;
        case MessageBoxIcon::QUESTION_ICON:
            icon_enum = pfd::icon::question;
            break;
    }

    pfd::notify("Baguette", content, icon_enum);
}

std::vector<std::string> openFile(const std::string& title, const std::string& initial_path,
                                  std::vector<std::string> filters, FileOptions options) {
    return pfd::open_file(title, initial_path, std::move(filters), pfd::opt(options)).result();
}

std::string saveFile(const std::string& title, const std::string& initial_path, std::vector<std::string> filters,
                     FileOptions options) {
    return pfd::save_file(title, initial_path, std::move(filters), pfd::opt(options)).result();
}

}  // namespace luhsoccer
