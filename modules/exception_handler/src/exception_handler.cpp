#include <string>
#include <fmt/format.h>
#include <fstream>

#include "exception_handler/exception_handler.hpp"
#include "logger/logger.hpp"
#include "utils/utils.hpp"

#define BOOST_STACKTRACE_USE_BACKTRACE
#if defined(__linux__)
#include "backtrace.h"
#endif
#include <boost/stacktrace.hpp>

namespace luhsoccer::exception_handler {

/**
 * @brief Logs the given string view to the logger and file
 *
 * @param message The message as a strng view
 * @param logger A Logger
 * @param stream A file stream
 */
void log(std::string_view message, logger::Logger &logger, std::ofstream &stream) {
    LOG_ERROR(logger, message);

    if (stream.is_open()) {
        stream << message << "\n";
    }
}

/**
 * @brief A function which creates and sets the terminate funciton of the program
 * The terminate function is called when a program aborts.
 */
void setTerminateHandler() {
    std::set_terminate([]() -> void {
        // get path of traceback file and open/create it
        auto traceback_path = getBaguetteDirectory();
        traceback_path += "/traceback.dump";
        std::ofstream file(traceback_path.c_str(), std::ios::app);

        logger::Logger logger("terminate_handler");

        // Create flags for permission checking
        bool file_opened = file.is_open();
        bool file_exists = true;
        if (!file_opened) {
            file.close();
            file_exists = std::filesystem::is_regular_file(traceback_path);
        }

        // add a seperator to the traceback File
        const auto time = std::chrono::system_clock::now();
        log(fmt::format("\n\n******************************* {} *******************************", time), logger, file);

        // handle exception
        log("Uncaught exception! Terminating baguette...", logger, file);

        try {
            // rethrow the uncaught exception so we can further analyze it
            std::rethrow_exception(std::current_exception());
        } catch (const std::exception &e) {
            log(fmt::format("Exception Type: {}", typeid(e).name()), logger, file);
            log(fmt::format("Exception: {}", e.what()), logger, file);
        } catch (...) {
            log(fmt::format("Non-Exception Type: {}", typeid(std::current_exception()).name()), logger, file);
            log("Something other than an exception was thrown", logger, file);
        }

        log(fmt::format("errno {}:{}", errno, std::strerror(errno)), logger, file);

        // Flush already written infos in case stacktrace fails
        file.flush();

        // Write traceback to logger and dump-file
        const auto stacktrace = boost::stacktrace::stacktrace();
        log("backtrace:", logger, file);
        if (file.is_open()) {
            file << stacktrace;
        }
        LOG_ERROR(logger, "{}", stacktrace);

        file.flush();
        file.close();

        // Write info text
        LOG_ERROR(logger, "************************************************************");
        LOG_ERROR(logger, "************************************************************");
        LOG_ERROR(logger, "BAGUETTE CRASHED");
        if (!file_opened) {
            LOG_ERROR(logger, "Traceback file could not be generated!");

            if (file_exists) {
                LOG_ERROR(logger, "Make sure to delete the following file:");
                LOG_ERROR(logger, "'{}'", traceback_path.string());
            } else {
                LOG_ERROR(logger, "Check permissions of following directory:");
                LOG_ERROR(logger, "{}", traceback_path.parent_path());
            }
            LOG_ERROR(logger, "Please report this incident to the Software Department");

        } else {
            LOG_ERROR(logger, "Find the traceback file at");
            LOG_ERROR(logger, "{}", traceback_path);
            LOG_ERROR(logger, "Please provider this file to the Software Department");
        }

        std::abort();
    });
}

}  // namespace luhsoccer::exception_handler

// /**
//  * @brief A Struct that is passed to the backtrace-print fucntions
//  */
// struct PrintData {
//     PrintData(std::ofstream &&stream, const logger::Logger &logger) : stream(std::move(stream)), logger(logger) {}

//     std::ofstream stream;
//     logger::Logger logger;
// };

// /**
//  * @brief Error Callback which is called if there is an error with the backtrace print
//  *
//  * @param data The PrintData Struct containing the file stream
//  * @param msg Error message
//  * @param errno_var Errno value
//  */
// void errorCallback(void *data, const char *msg, int errno_var) {
//     auto pdata = static_cast<PrintData *>(data);
//     std::cerr << "backtrace error" << std::endl;

//     pdata->stream << fmt::format("backtrace error; errno: {}; Message: {}", errno_var, std::strerror(errno_var))
//                   << std::endl;
//     if (msg != nullptr) {
//         pdata->stream << "Msg: " << msg << std::endl;
//     }
// }

// /**
//  * @brief The Function which is reliable for printing the individual backtace lines
//  *
//  * @param data The PrintData struct which contains the traceback-file stream
//  * @param program_counter The backtrace-Program-counter
//  * @param raw_filename The filename for the current line
//  * @param line_number The line number
//  * @param function The function for the backtrace line
//  * @return int Always 0
//  */
// int printCallback(void *data, unsigned long program_counter, const char *raw_filename, int line_number,
//                   const char *function) {
//     auto pdata = static_cast<PrintData *>(data);

//     std::string method_name;
//     if (function == nullptr) {
//         method_name = "[unknown_method]";
//     } else {
// #if defined(_MSVC_VER)
//         // @todo demangle in MSVC
//         method_name = function;
// #else
//         int status = 0;
//         char *demangeld_name = abi::__cxa_demangle(function, 0, 0, &status);
//         if (demangeld_name == nullptr) {
//             method_name = "[unknown_method]";

//         } else {
//             // copy the underlying chars
//             method_name = demangeld_name;

//             // The method abi::__cxa_demangle returns a char* that needs to be freed (allocated with malloc)
//             // Since we copy the underlying string with the copy constructor of std::string and instantly free the
//             // memory this should be fine
//             // NOLINTNEXT (cppcoreguidelines-no-malloc, cppcoreguidelines-owning-memory)
//             free(static_cast<void *>(demangeld_name));
//         }
// #endif
//     }

//     std::string file_name = raw_filename == nullptr ? "[unknown_file]" : raw_filename;

//     std::string backtrace_line =
//         fmt::format("pc: {:x}; {}\n\t{}:{}", program_counter, method_name, file_name, line_number);

//     if (pdata == nullptr) {
//         std::cerr << backtrace_line << std::endl;
//     } else {
//         // write backtrace line to cerr and to the traceback file
//         LOG_ERROR(pdata->logger, "{}", backtrace_line);
//         pdata->stream << backtrace_line << std::endl;
//     }

//     return 0;
// }

// /**
//  * @brief The method which initialtes the backtrace-print process
//  *
//  * @param logger A Logger
//  * @param stream The traceback file stream
//  */
// void printBacktrace(logger::Logger &logger, std::ofstream &stream) {
//     auto data = PrintData(std::move(stream), logger);

//     struct backtrace_state *state = backtrace_create_state(nullptr, 0, errorCallback, &data);
//     if (state == nullptr) {
//         log("Could not generate Backtrace Info!", logger, stream);
//         stream.flush();
//     }

//     backtrace_full(state, 0, printCallback, errorCallback, static_cast<void *>(&data));
// }
