#include "game_controller_interface_impl.hpp"
#include "process.hpp"
#include "time/time.hpp"

namespace luhsoccer::game_controller_interface {

GameControllerInterfaceImpl::GameControllerInterfaceImpl(std::string gc_path)
    : path_to_gc(std::move(gc_path)), gc_logger("game_controller"){};

GameControllerInterfaceImpl::~GameControllerInterfaceImpl() { this->stopGameController(); }

bool GameControllerInterfaceImpl::startGameController() {
    /*
    Construct the command that is executed
    -timeAcquisitionMode ci -> In CI mode out software can send time & field data to the game controller
    -publishAddress 0 -> by unsetting the publish adress the game controller doesnt publish its data over the network or
    via multicast
    */

    if (this->gc_proc.has_value()) {
        return true;
    }

    const std::string gc_arguments = "-timeAcquisitionMode ci -publishAddress 0";
    const std::string gc_process_input = this->path_to_gc + " " + gc_arguments;

    // create empty environment / Dont pass any env-variables
    TinyProcessLib::Process::environment_type env = {};

    // create path for program
    std::string path = "";

    // print stdout of the game controller to the logger
    const auto stdout_reader = [this](const char* bytes, std::size_t n) {
        std::string_view gc_output{bytes, n};
        LOG_INFO(this->gc_logger, "{}", gc_output);
    };

    // create & start process
    this->gc_proc.emplace(gc_process_input, path, env, stdout_reader);

    // wait for a short ammount of time so that the process has time to **try** to start
    // This Delay should be ok, because it should only be called through luhvize's GUI
    constexpr double PROCESS_START_MARGIN = 0.15;
    const time::Duration start_margin = time::Duration(PROCESS_START_MARGIN);
    std::this_thread::sleep_for(start_margin);

    // if the process already exited it didnt start successfully
    int exit_status = 0;
    const bool process_ended = this->gc_proc->try_get_exit_status(exit_status);

    if (process_ended) {
        LOG_WARNING(this->gc_logger, "Could not start Game Controller! (exit status {})", exit_status);
        this->gc_proc = std::nullopt;
        return false;
    } else {
        LOG_INFO(this->gc_logger, "Game Controller started");
        return true;
    }
}

bool GameControllerInterfaceImpl::stopGameController() {
    if (!this->gc_proc.has_value()) {
        return true;
    }

    // try to kill the process
    this->gc_proc->kill();

    constexpr double WAIT_TIMEOUT = 2;
    int exit_status = 0;
    bool wait_succeeded = true;
    bool already_force_killed = false;
    const auto start_time = time::now();

    // wait until it exits (or timeout if it doesnt exit)
    while (!this->gc_proc->try_get_exit_status(exit_status)) {
        const auto current_time = time::now();
        const double diff = current_time.asSec() - start_time.asSec();

        // this only works on linux; after a second we try to foce-kill the process
        if (!already_force_killed && diff >= WAIT_TIMEOUT / 2) {
            this->gc_proc->kill(true);
            already_force_killed = true;
        }

        if (diff >= WAIT_TIMEOUT) {
            wait_succeeded = false;
            break;
        }
    }

    if (wait_succeeded) {
        if (exit_status) {
            LOG_WARNING(this->gc_logger, "Game Controller terminated with code {}", exit_status);
        } else {
            LOG_INFO(this->gc_logger, "Game Controller terminated sucessfully!", exit_status);
        }
        this->gc_proc = std::nullopt;
    } else {
        LOG_ERROR(this->gc_logger, "Could not stop Game Controller! It might still be running in the background!");
    }

    return wait_succeeded;
}

}  // namespace luhsoccer::game_controller_interface