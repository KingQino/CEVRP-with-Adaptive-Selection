#include "../include/command_line.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_set>

namespace fs = std::filesystem;

namespace {

std::string normalize_key(std::string key) {
    while (!key.empty() && key.front() == '-') {
        key.erase(key.begin());
    }
    return key;
}

std::string lowercase(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

bool starts_with_option_prefix(const std::string& value) {
    return !value.empty() && value.front() == '-';
}

LocalSearchIntensity parse_local_search_intensity(const std::string& value) {
    const std::string normalized = lowercase(value);
    if (normalized == "weak") {
        return LocalSearchIntensity::Weak;
    }
    if (normalized == "medium") {
        return LocalSearchIntensity::Medium;
    }
    if (normalized == "strong") {
        return LocalSearchIntensity::Strong;
    }
    throw std::invalid_argument(
        "ls must be one of: weak, medium, strong");
}

}  // namespace

CommandLine::CommandLine(int argc, char* argv[]) {
    fs::path executablePath = fs::absolute(argv[0]);
    std::error_code error;
    const fs::path canonicalPath = fs::weakly_canonical(executablePath, error);
    if (!error) {
        executablePath = canonicalPath;
    }
    projectRoot = executablePath.parent_path().parent_path();

    if (argc > 1 && !starts_with_option_prefix(argv[1])) {
        legacyMode = true;
        if (argc < 4 || argc > 5) {
            throw std::invalid_argument(
                "legacy usage: ./Run <instance> <stp> <mth> [weak|medium|strong]");
        }
        arguments["ins"] = argv[1];
        arguments["stp"] = argv[2];
        arguments["mth"] = argv[3];
        if (argc == 5) {
            arguments["ls"] = argv[4];
        }
        return;
    }

    for (int index = 1; index < argc;) {
        const std::string key = normalize_key(argv[index]);
        if (key == "h" || key == "help") {
            helpRequested = true;
            ++index;
            continue;
        }
        if (index + 1 >= argc) {
            throw std::invalid_argument(
                "missing value for command-line option '" + std::string(argv[index]) + "'");
        }
        arguments[key] = argv[index + 1];
        index += 2;
    }
}

void CommandLine::parse_parameters(Parameters& params) const {
    static const std::unordered_set<std::string> knownOptions = {
        "ins",
        "log",
        "stp",
        "mth",
        "seed",
        "pop_size",
        "mutation_prob",
        "mutation_ind_prob",
        "tournament_size",
        "ls",
        "parent_pool_ratio",
        "quality_ratio",
        "verified_upper_ratio",
        "pure_immigrant_ratio",
        "gamma",
    };
    for (const auto& [key, value] : arguments) {
        (void)value;
        if (knownOptions.find(key) == knownOptions.end()) {
            throw std::invalid_argument("unknown command-line option '-" + key + "'");
        }
    }

    params.dataPath = (projectRoot / "data").string();
    params.statsPath = (projectRoot / "stats").string();
    params.instance = get_string("ins", params.instance);
    params.enableLogging = get_bool("log", legacyMode ? true : params.enableLogging);
    params.stopCriteria = get_int("stp", params.stopCriteria);
    params.enableMultithreading = get_bool("mth", params.enableMultithreading);
    params.seed = get_int("seed", params.seed);
    params.popSize = get_int("pop_size", params.popSize);
    params.mutationProb = get_double("mutation_prob", params.mutationProb);
    params.mutationIndProb = get_double(
        "mutation_ind_prob",
        params.mutationIndProb);
    params.tournamentSize = get_int("tournament_size", params.tournamentSize);
    params.localSearchIntensity = parse_local_search_intensity(
        get_string("ls", "strong"));
    params.parentPoolRatio = get_double(
        "parent_pool_ratio",
        params.parentPoolRatio);
    params.qualityRatio = get_double("quality_ratio", params.qualityRatio);
    params.verifiedUpperRatio = get_double(
        "verified_upper_ratio",
        params.verifiedUpperRatio);
    params.pureImmigrantRatio = get_double(
        "pure_immigrant_ratio",
        params.pureImmigrantRatio);
    params.gamma = get_double("gamma", params.gamma);
}

bool CommandLine::help_requested() const {
    return helpRequested;
}

void CommandLine::display_help() {
    std::cout
        << "Usage: ./Run [options]\n"
        << "  -ins <file>                    Instance filename\n"
        << "  -seed <int>                    Random seed (default: 0)\n"
        << "  -stp <1|2>                     1: max evals, 2: max time (default: 1)\n"
        << "  -mth <0|1>                     0: one seeded run, 1: ten parallel runs (default: 0)\n"
        << "  -log <0|1>                     Write evolution/LS/solution logs (default: 0)\n"
        << "  -pop_size <int>                Population size (default: 100)\n"
        << "  -mutation_prob <double>        Offspring mutation probability (default: 0.5)\n"
        << "  -mutation_ind_prob <double>    Per-gene mutation probability (default: 0.2)\n"
        << "  -tournament_size <int>         Parent tournament size (default: 2)\n"
        << "  -ls <weak|medium|strong>        Local-search intensity (default: strong)\n"
        << "  -parent_pool_ratio <double>    Parent-pool/population ratio (default: 0.10)\n"
        << "  -quality_ratio <double>        Quality share in parent pool (default: 0.50)\n"
        << "  -verified_upper_ratio <double> verifiedBest x P_upper share (default: 0.05)\n"
        << "  -pure_immigrant_ratio <double> Pure immigrant share (default: 0.10)\n"
        << "  -gamma <double>                Follower trigger ratio (default: 1.02)\n"
        << "\nLegacy syntax remains accepted:\n"
        << "  ./Run <instance> <stp> <mth> [weak|medium|strong]\n";
}

int CommandLine::get_int(const std::string& key, int defaultValue) const {
    const auto iterator = arguments.find(key);
    if (iterator == arguments.end()) {
        return defaultValue;
    }
    std::size_t parsedLength = 0;
    const int value = std::stoi(iterator->second, &parsedLength);
    if (parsedLength != iterator->second.size()) {
        throw std::invalid_argument("invalid integer for -" + key + ": " + iterator->second);
    }
    return value;
}

double CommandLine::get_double(const std::string& key, double defaultValue) const {
    const auto iterator = arguments.find(key);
    if (iterator == arguments.end()) {
        return defaultValue;
    }
    std::size_t parsedLength = 0;
    const double value = std::stod(iterator->second, &parsedLength);
    if (parsedLength != iterator->second.size() || !std::isfinite(value)) {
        throw std::invalid_argument("invalid number for -" + key + ": " + iterator->second);
    }
    return value;
}

std::string CommandLine::get_string(
    const std::string& key,
    const std::string& defaultValue) const {
    const auto iterator = arguments.find(key);
    return iterator == arguments.end() ? defaultValue : iterator->second;
}

bool CommandLine::get_bool(const std::string& key, bool defaultValue) const {
    const auto iterator = arguments.find(key);
    if (iterator == arguments.end()) {
        return defaultValue;
    }
    const std::string value = lowercase(iterator->second);
    if (value == "1" || value == "true" || value == "yes") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no") {
        return false;
    }
    throw std::invalid_argument("invalid boolean for -" + key + ": " + iterator->second);
}
