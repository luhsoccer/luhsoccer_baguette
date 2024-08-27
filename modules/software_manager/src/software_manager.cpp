#include "config_provider/config_store_main.hpp"
#include "config/simulation_interface_config.hpp"

#include "software_manager/software_manager.hpp"

#include "software.hpp"

#include "utils/utils.hpp"

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>

#include <fstream>

#include <fmt/ranges.h>

#include "cmrc/cmrc.hpp"

#ifdef __linux__
#include <sys/prctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <sys/wait.h>
#endif

#ifdef _WIN32
#define NOGDI
#include <Windows.h>
#endif

CMRC_DECLARE(software_manager);

namespace luhsoccer::software_manager {

namespace {

bool existsEmbeddedFile(const std::string& path) { return cmrc::software_manager::get_filesystem().exists(path); }

std::string getEmbeddedFileContent(const std::string& path) {
    auto file = cmrc::software_manager::get_filesystem().open(path);
    std::string data{file.begin(), file.end()};
    return data;
}

std::string getSimulatorRealism() {
    auto make_path = [](const std::string& name) { return "simulator_presets/realism/" + name + ".txt"; };
    std::string path =
        make_path(config_provider::ConfigProvider::getConfigStore().simulation_interface_config.simulator_realism);
    if (existsEmbeddedFile(path)) {
        return getEmbeddedFileContent(path);
    } else {
        return getEmbeddedFileContent(make_path("RC2021"));
    }
}

std::string getSimulatorGeometry() {
    auto make_path = [](const std::string& name) { return "simulator_presets/geometry/" + name + ".txt"; };
    std::string path =
        make_path(config_provider::ConfigProvider::getConfigStore().simulation_interface_config.simulator_geometry);
    if (existsEmbeddedFile(path)) {
        return getEmbeddedFileContent(path);
    } else {
        return getEmbeddedFileContent(make_path("2020"));
    }
}

std::string_view getComponentDownloadUrl(SoftwareComponent component) {
#if __linux__ && __x86_64__
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return software::game_controller::DOWNLOAD_LINUX_AMD64;
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return software::simulation::DOWNLOAD_LINUX_AMD64;
        default:
            break;
    }
#elif _WIN32
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return software::game_controller::DOWNLOAD_WINDOWS_AMD64;
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return software::simulation::DOWNLOAD_WINDOWS_AMD64;
        default:
            break;
    }
#else
#error "The current platform is not supported or not defined."
#endif

    return "UNKNOWN";
}

std::string_view getComponentFileName(SoftwareComponent component) {
#if __linux__ && __x86_64__
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return "game_controller";
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return "er_sim";
        default:
            break;
    }
#elif _WIN32
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return "game_controller.exe";
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return "er_sim.exe";
        default:
            break;
    }
#else
#error "Unknown platform"
#endif

    return "UNKNOWN";
}

std::filesystem::path getComponentPath(SoftwareComponent component) {
    return getBaguetteDirectory() / "software" / format_as(component) / getComponentFileName(component);
}

std::vector<std::string> getComponentArgs(SoftwareComponent component) {
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return {"-timeAcquisitionMode", "ci", "-publishAddress", "0", "-address", "localhost:26782"};
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM: {
            auto geometry_path = getBaguetteDirectory() / "software" / format_as(component) / "geometry.txt";
            auto realism_path = getBaguetteDirectory() / "software" / format_as(component) / "realism.txt";

            return {"--loadSavedState",     "--disableMulticast", "--absolutePath",     "--geometry",
                    geometry_path.string(), "--realism",          realism_path.string()};
        }
        default:
            break;
    }

    return {};
}

std::string_view getComponentVersion(SoftwareComponent component) {
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return luhsoccer::software_manager::software::game_controller::VERSION;
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return luhsoccer::software_manager::software::simulation::VERSION;
        default:
            break;
    }

    return "UNKNOWN";
}
}  // namespace

std::string_view format_as(SoftwareComponent component) {
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return "GameController";
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return "ErSim";
        default:
            break;
    }

    return "UNKNOWN";
}

std::string_view getEnvName(SoftwareComponent component) {
    switch (component) {
        case luhsoccer::software_manager::SoftwareComponent::GAME_CONTROLLER:
            return "GAME_CONTROLLER";
        case luhsoccer::software_manager::SoftwareComponent::ER_SIM:
            return "ER_SIM";
        default:
            break;
    }

    return "UNKNOWN";
}

#if __linux__
class NativeAddons {};
#elif _WIN32
class NativeAddons {
   public:
    NativeAddons() : job_handle(CreateJobObject(nullptr, nullptr)) {
        // Based on https://stackoverflow.com/questions/53208/how-do-i-automatically-destroy-child-processes-in-windows
        if (job_handle == nullptr) {
            ::MessageBoxA(nullptr, "Failed to create job object for native process handling", "Baguette", MB_ICONERROR);
        } else {
            JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli = {0};
            jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            if (!::SetInformationJobObject(job_handle, JobObjectExtendedLimitInformation, &jeli, sizeof(jeli))) {
                ::MessageBoxA(nullptr, "Failed to set information for job object for native process handling",
                              "Baguette", MB_ICONERROR);
            }
        }
    }

    std::shared_mutex mutex;
    HANDLE job_handle;
    std::unordered_map<unsigned int, HANDLE> pid_to_handle;
};
#endif

SoftwareManager::SoftwareManager() : native_addons(std::make_unique<NativeAddons>()) {
    for (const auto& component : getComponents()) {
        component_states[component] = {false, false, 0};
    }
}

SoftwareManager::~SoftwareManager() = default;

void SoftwareManager::startComponent(SoftwareComponent component) {
    std::shared_lock lock(state_mutex);
    if (component_states.at(component).running) {
        logger.info("Component {} is already running", component);
    } else {
        component_states[component].should_run = true;
    }
}

std::vector<SoftwareComponent> SoftwareManager::getComponents() const {
    return {SoftwareComponent::GAME_CONTROLLER, SoftwareComponent::ER_SIM};
}

ComponentState SoftwareManager::getComponentState(SoftwareComponent component) const {
    std::shared_lock lock(state_mutex);
    return component_states.at(component);
}

void SoftwareManager::setup(event_system::EventSystem& event_system) {
    event_system.registerEventHandler<event_system::TimerEvent1Hz>(
        [this](const event_system::EventContext<event_system::TimerEvent1Hz>& /*event*/) { this->update(); });
    for (const auto& component : getComponents()) {
        auto env_name = fmt::format("BAGUETTE_START_{}", getEnvName(component));
        if (std::getenv(env_name.c_str()) != nullptr) {
            startComponent(component);
        }
    }
}

void SoftwareManager::update() {
    std::shared_lock lock(state_mutex);

    // Check for state: To-Be-Started, To-Be-Stopped and Running
    for (auto& [component, state] : component_states) {
        if (state.should_run && !state.running) {  // To-Be-Started state
            logger.info("Starting component {}", component);
            if (!hasComponent(component)) {
                downloadComponent(component);
            }

            if (component == SoftwareComponent::ER_SIM) {
                auto write_content = [](const std::string& file_path, const std::string& content) {
                    std::ofstream file(file_path, std::ios::out | std::ios::trunc);
                    file << content;
                    file.close();
                };

                auto geometry_path = getBaguetteDirectory() / "software" / format_as(component) / "geometry.txt";
                auto realism_path = getBaguetteDirectory() / "software" / format_as(component) / "realism.txt";
                write_content(geometry_path.string(), getSimulatorGeometry());
                write_content(realism_path.string(), getSimulatorRealism());
            }

            unsigned int pid = execFile(getComponentPath(component).string(), getComponentArgs(component));

            {
                lock.unlock();
                std::unique_lock read_lock(state_mutex);
                state.pid = pid;
                if (pid == 0) {
                    logger.error("Failed to start component {}", component);
                    state.should_run = false;
                } else {
                    logger.info("Started component {} with pid {}", component, pid);
                    state.running = true;
                }
                read_lock.unlock();
                // The mutex needs to be locked again
                // since we're looping through the components
                lock.lock();
            }
        } else if (!state.should_run && state.running) {  // To-Be-Stopped state
            logger.info("Stopping component {}", component);
            if (stopSubprocess(state.pid)) {
                {
                    lock.unlock();
                    std::unique_lock read_lock(state_mutex);
                    state.pid = 0;
                    state.running = false;
                    read_lock.unlock();
                    // The mutex needs to be locked again
                    // since we're looping through the components
                    lock.lock();
                }
                logger.info("Stopped component {}", component);
            } else {
                logger.error("Failed to stop component {}", component);
            }
        } else if (state.running && state.should_run) {  // Running state, check if the process is still alive
            if (!isAlive(state.pid)) {
                logger.error("Component {} with pid {} is not alive anymore", component, state.pid);
                {
#if _WIN32
                    std::unique_lock native_lock(native_addons->mutex);
                    ::CloseHandle(native_addons->pid_to_handle[state.pid]);
                    native_addons->pid_to_handle.erase(state.pid);
#endif
                    lock.unlock();
                    std::unique_lock read_lock(state_mutex);
                    state.running = false;
                    state.should_run = false;
                    state.pid = 0;
                    read_lock.unlock();
                    // The mutex needs to be locked again
                    // since we're looping through the components
                    lock.lock();
                }
            }
        }
    }
}

bool SoftwareManager::isRunning(SoftwareComponent component) {
    std::shared_lock lock(state_mutex);
    return component_states.at(component).running;
}

bool SoftwareManager::shouldRun(SoftwareComponent component) {
    std::shared_lock lock(state_mutex);
    return component_states.at(component).should_run;
}

bool SoftwareManager::hasComponent(SoftwareComponent component) {
    auto version_file = getBaguetteDirectory() / "software" / format_as(component) / "VERSION";
    auto file = getComponentPath(component);
    logger.info("Checking if component {} exists in {} and {}", component, version_file.string(), file.string());

    if (!std::filesystem::exists(version_file)) {
        // The version file does not exist, even if the file exists we clean the state and download the
        // component
        logger.debug("Version file does not exist, expected: {}", version_file.string());
        deleteComponent(component);
        return false;
    }
    std::ifstream version_stream(version_file);
    std::string version;
    version_stream >> version;
    version_stream.close();

    std::string our_version = std::string(getComponentVersion(component));

    if (version != our_version) {
        // The version file exists but the version is not the same, we clean the state and download the
        // component
        logger.debug("Version file exists but the version is not the same, expected: {} got: {}", our_version, version);
        deleteComponent(component);
        return false;
    }

    if (!std::filesystem::exists(file)) {
        // The file does not exist, we clean the state and download the component
        logger.debug("File does not exist, expected: {}", file.string());
        deleteComponent(component);
        return false;
    }

    return true;
}

void SoftwareManager::deleteComponent(SoftwareComponent component) {
    auto component_directory = getBaguetteDirectory() / "software" / format_as(component);

    if (std::filesystem::exists(component_directory)) {
        std::filesystem::remove_all(component_directory);
    }
}

void SoftwareManager::downloadComponent(SoftwareComponent component) {
    logger.info("Missing component {} start downloading", component);
    auto file_path = getComponentPath(component);
    std::string url(getComponentDownloadUrl(component));
    std::filesystem::create_directories(file_path.parent_path());
    if (download(url, file_path.string())) {
        // Write the downloaded version to the version file
        // Every baguette version only guarantees the compatibility with the version that was shipped
        // We delete the downloaded component if there is a mismatch
        std::ofstream version_file(getBaguetteDirectory() / "software" / format_as(component) / "VERSION");
        version_file << std::string(getComponentVersion(component));
        version_file.close();
    } else {
        logger.error("Failed to download component {}", component);
    }
}

void SoftwareManager::stopComponent(SoftwareComponent component) {
    std::unique_lock lock(state_mutex);
    if (component_states.at(component).running) {
        component_states[component].should_run = false;
    }
}

bool SoftwareManager::download(const std::string& url, const std::string& file_name) {
    try {
        curlpp::Cleanup cleaner;
        curlpp::Easy request;

        std::ofstream response_stream;
        response_stream.open(file_name, std::ios::out | std::ios::trunc | std::ios::binary);

        request.setOpt<curlpp::options::Url>(url);
        request.setOpt<curlpp::options::FollowLocation>(true);
        request.setOpt<curlpp::options::WriteStream>(&response_stream);

        request.setOpt<curlpp::options::ProgressFunction>([&](double total, double downloaded, double, double) {
            time::TimePoint current_time = time::now();
            constexpr int KILOBYTE = 1024;
            constexpr std::chrono::milliseconds DELAY(500);
            auto downloaded_us = static_cast<unsigned int>(downloaded) / KILOBYTE;
            if (current_time - last_time > DELAY && downloaded_us > 0) {
                last_time = current_time;
                logger.info("Downloaded: {} kb of {} kb", downloaded_us, static_cast<unsigned int>(total) / KILOBYTE);
            }
            return 0;
        });
        request.setOpt<curlpp::options::NoProgress>(false);
        request.perform();

        response_stream.close();
        std::filesystem::permissions(file_name, std::filesystem::perms::owner_all);
        logger.info("Successfully downloaded file {} to {}", url, file_name);
        return true;
    } catch (curlpp::LogicError& e) {
        logger.error("Logic error while downloading file {}: {}", file_name, e.what());
    } catch (curlpp::RuntimeError& e) {
        logger.error("Runtime error while downloading file {}: {}", file_name, e.what());
    } catch (std::exception& e) {
        logger.error("Exception while downloading file {}: {}", file_name, e.what());
    } catch (...) {
        logger.error("Error while downloading file {}: Unknown error", file_name);
        // Handle exception
    }

    return false;
}

bool SoftwareManager::stopSubprocess(unsigned int pid) {
#if __linux__ && __x86_64__
    int ret = kill(pid, SIGTERM);  // NOLINT(cppcoreguidelines-narrowing-conversions,bugprone-narrowing-conversions)
    if (ret == -1) {
        logger.error("Failed to kill process with pid {}", pid);
        return false;
    }
#elif _WIN32
    if (::TerminateProcess(native_addons->pid_to_handle[pid], 0)) {
        return true;
    }
    logger.error("Failed to terminate process with pid {}", pid);
    return false;
#else
#error "The current platform is not supported or not defined."
#endif

    return true;
}

bool SoftwareManager::isAlive(unsigned int pid) {
    if (pid == 0) {
        return false;
    }
#if __linux__ && __x86_64__
    int status = 0;
    // NOLINTNEXTLINE(cppcoreguidelines-narrowing-conversions,bugprone-narrowing-conversions)
    int ret = waitpid(pid, &status, WNOHANG);
    return ret == 0;
#elif _WIN32
    DWORD exit_code = 0;
    if (::GetExitCodeProcess(this->native_addons->pid_to_handle[pid], &exit_code)) {
        return exit_code == STILL_ACTIVE;
    }
    return false;
#else
#error "The current platform is not supported or not defined."
#endif
}

unsigned int SoftwareManager::execFile(const std::string& file, const std::vector<std::string>& args) {
    std::string working_dir = getBaguetteDirectory().string();
    logger.info("Starting process to execute file {} with args {}", file, args);
#if __linux__ && __x86_64__
    // Very syscall and linux specific code so we disable the linter
    // NOLINTBEGIN

    // Here we check if the file is executable
    // It should be executable by default because we set the flag before
    // But if the user has some weird things going on we should check
    if (access(file.c_str(), X_OK)) {
        logger.error("File {} is not executable. This is a user error, try delete the file.", file);
        return 0;
    }

    // Based from:
    // https://stackoverflow.com/questions/284325/how-to-make-child-process-die-after-parent-exits/36945270#36945270
    pid_t pid_before_fork = getpid();
    pid_t pid = fork();

    if (pid == -1) {
        logger.info("Failed to fork process to execute file {}", file);
        return 0;
    }

    if (pid == 0) {
        // Kill the child process if the parent process dies
        int ret = prctl(PR_SET_PDEATHSIG, SIGTERM);
        if (ret == -1) {
            std::exit(1);
            return -1;
        }

        // Check if parent is still alive
        if (getppid() != pid_before_fork) {
            std::exit(1);
            return 0;
        }

        std::vector<char*> c_args;

        c_args.push_back(const_cast<char*>(file.c_str()));

        for (const auto& arg : args) {
            // This is okay here because execve will not modify the strings and only copy the content
            c_args.push_back(const_cast<char*>(arg.c_str()));
        }

        c_args.push_back(nullptr);

        // Disable the stdout and stderr of the child process
        // Based of https://stackoverflow.com/questions/26453624/hide-terminal-output-from-execve
        int fd = open("/dev/null", O_WRONLY);
        dup2(fd, 1);
        dup2(fd, 2);
        close(fd);

        // Set the working directory
        chdir(working_dir.c_str());

        int ret_exe = execve(file.c_str(), c_args.data(), nullptr);
        if (ret_exe == -1) {
            std::exit(1);
        }
        // NOLINTEND
    }
    return pid;
#elif _WIN32
    // Very syscall and windows specific code so we disable the linter
    // NOLINTBEGIN
    // Based on
    // https://stackoverflow.com/questions/53208/how-do-i-automatically-destroy-child-processes-in-windows
    STARTUPINFO si = {
        .cb = sizeof(STARTUPINFO),
        .dwFlags = STARTF_USESTDHANDLES,
        .hStdInput = nullptr,
        .hStdOutput = nullptr,
        .hStdError = nullptr,
    };
    PROCESS_INFORMATION pi;

    std::string quoted_file = "\"" + file + "\"";

    std::string args_str;

    args_str += quoted_file;
    for (const auto& arg : args) {
        args_str += " " + arg;
    }

    if (::CreateProcess(nullptr, const_cast<char*>(args_str.c_str()), nullptr, nullptr, true, 0, nullptr,
                        working_dir.c_str(), &si, &pi)) {
        std::unique_lock lock(native_addons->mutex);
        ::AssignProcessToJobObject(native_addons->job_handle, pi.hProcess);

        ::CloseHandle(pi.hThread);
        native_addons->pid_to_handle[pi.dwProcessId] = pi.hProcess;
        return pi.dwProcessId;
    } else {
        logger.error("Failed to create process to execute file {}", file);
    }

    // NOLINTEND
#else
#error "The current platform is not supported or not defined.";
#endif
}

}  // namespace luhsoccer::software_manager