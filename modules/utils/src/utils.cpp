// Here is place for your own code
#include "utils/utils.hpp"
#include <filesystem>

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

}  // namespace luhsoccer
