#ifndef COMMAND_LINE_HPP
#define COMMAND_LINE_HPP

#include <filesystem>
#include <string>
#include <unordered_map>

#include "parameters.hpp"

class CommandLine {
public:
    CommandLine(int argc, char* argv[]);

    void parse_parameters(Parameters& params) const;
    [[nodiscard]] bool help_requested() const;
    static void display_help();

private:
    std::unordered_map<std::string, std::string> arguments;
    std::filesystem::path projectRoot;
    bool helpRequested = false;
    bool legacyMode = false;

    [[nodiscard]] int get_int(const std::string& key, int defaultValue) const;
    [[nodiscard]] double get_double(const std::string& key, double defaultValue) const;
    [[nodiscard]] std::string get_string(
        const std::string& key,
        const std::string& defaultValue) const;
    [[nodiscard]] bool get_bool(const std::string& key, bool defaultValue) const;
};

#endif
