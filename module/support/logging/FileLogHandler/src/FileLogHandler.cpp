#include "FileLogHandler.hpp"

#include <filesystem>

#include "extension/Configuration.hpp"

#include "utility/strutil/ansi.hpp"
#include "utility/support/evil/pure_evil.hpp"

namespace module::support::logging {

    using NUClear::message::LogMessage;
    using NUClear::message::ReactionStatistics;

    using extension::Configuration;

    using utility::strutil::Colour;

    FileLogHandler::FileLogHandler(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , log_file_name("/home/nubots/log")
        , log_file(log_file_name, std::ios_base::out | std::ios_base::app | std::ios_base::ate) {

        on<Configuration>("FileLogHandler.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FileLogHandler.yaml
            log_file_name = std::filesystem::path(config["log_file"].as<std::string>());

            max_size = config["max_size"].as<int>();

            if (log_file.is_open()) {
                log_file.close();
            }

            log_file.open(log_file_name, std::ios_base::out | std::ios_base::app | std::ios_base::ate);

            log_file << "\n*********************************************************************\n" << std::endl;
        });

        on<Shutdown>().then([this] {
            if (log_file.is_open()) {
                log_file.close();
            }
        });

        stats_reaction = on<Trigger<ReactionStatistics>, Sync<FileLogHandler>>().then([this](const ReactionStatistics& stats) {
            if (stats.exception) {

                // Get our reactor name
                std::string reactor = stats.identifier[1];

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

#ifndef NDEBUG  // We have a cold hearted monstrosity that got built!

                // Print our exception detals
                log_file << reactor << " " << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                         << Colour::brightred << "Exception:"
                         << " " << Colour::brightred << utility::support::evil::exception_name << std::endl;

                // Print our stack trace
                for (auto& s : utility::support::evil::stack) {
                    log_file << "\t" << Colour::brightmagenta << s.file << ":" << Colour::brightmagenta << s.lineno
                             << " " << s.function << std::endl;
                }
#else
                try {
                    std::rethrow_exception(stats.exception);
                }
                catch (const std::exception& ex) {

                    std::string exceptionName = NUClear::util::demangle(typeid(ex).name());

                    log_file << reactor << " " << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                             << Colour::brightred << "Exception:"
                             << " " << Colour::brightred << exceptionName << " " << ex.what() << std::endl;
                }
                // We don't actually want to crash
                catch (...) {

                    log_file << reactor << " " << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                             << Colour::brightred << "Exception of unkown type" << std::endl;
                }
#endif
            }
        });

        logging_reaction = on<Trigger<LogMessage>, Sync<FileLogHandler>>().then([this](const LogMessage& message) {
            // Where this message came from
            std::string source = "";

            // If we know where this log message came from, we display that
            if (message.task && message.task->identifier.size() >= 2) {
                // Get our reactor name
                std::cout << message.task->identifier[1] << std::endl;
                std::string reactor = message.task->identifier[1];

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                // This is our source
                source = reactor + " "
                         + (message.task->identifier[0].empty() ? "" : "- " + message.task->identifier[0] + " ");
            }

            // Output the level
            switch (message.level) {
                case NUClear::TRACE: log_file << source << "TRACE: "; break;
                case NUClear::DEBUG: log_file << source << Colour::green << "DEBUG: "; break;
                case NUClear::INFO: log_file << source << Colour::brightblue << "INFO: "; break;
                case NUClear::WARN: log_file << source << Colour::yellow << "WARN: "; break;
                case NUClear::ERROR: log_file << source << Colour::brightred << "ERROR: "; break;
                case NUClear::FATAL: log_file << source << Colour::brightred << "FATAL: "; break;
            }

            // Output the message
            log_file << message.message << std::endl;
        });


        // This checks that we haven't reached the max_size
        log_check_handler = on<Every<5, std::chrono::seconds>, Sync<FileLogHandler>, Single>().then([this]() {
            int size = 0;
            for (auto& f : std::filesystem::recursive_directory_iterator(log_file_name.remove_filename())) {
                if (f.is_regular_file()) {
                    size += f.file_size();
                }
            }
            if (size >= max_size) {
                logging_reaction.unbind();
                stats_reaction.unbind();
                log_check_handler.unbind();
                log<NUClear::WARN>("FileLogHandler disabled - Maximum logging amount exceeded.");
                if (log_file.is_open()) {
                    log_file.close();
                }
            }
        });
    }
}  // namespace module::support::logging
