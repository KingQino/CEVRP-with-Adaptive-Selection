#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "command_line.hpp"
#include "parameters.hpp"

namespace {

CommandLine make_command_line(std::vector<std::string>& arguments) {
    std::vector<char*> argv;
    argv.reserve(arguments.size());
    for (auto& argument : arguments) {
        argv.push_back(argument.data());
    }
    return CommandLine(static_cast<int>(argv.size()), argv.data());
}

bool validation_fails(const Parameters& parameters, const std::string& expectedText) {
    try {
        parameters.validate();
    } catch (const std::invalid_argument& error) {
        return std::string(error.what()).find(expectedText) != std::string::npos;
    }
    return false;
}

}  // namespace

int main() {
    std::vector<std::string> namedArguments = {
        "build/command_line_test",
        "-ins", "F-n140-k5-s5.evrp",
        "-seed", "17",
        "-stp", "1",
        "-mth", "0",
        "-log", "0",
        "-pop_size", "120",
        "-mutation_prob", "0.25",
        "-mutation_ind_prob", "0.005",
        "-tournament_size", "3",
        "-ls", "medium",
        "-ls_policy", "static",
        "-parent_pool_ratio", "0.15",
        "-quality_ratio", "0.4",
        "-verified_upper_ratio", "0.10",
        "-pure_immigrant_ratio", "0.15",
        "-gamma", "1.05",
        "-elite_rho", "0.20",
        "-ls_depth", "d6",
        "-ls_cost_penalty", "0.05",
    };
    CommandLine namedCommandLine = make_command_line(namedArguments);
    Parameters namedParameters;
    namedCommandLine.parse_parameters(namedParameters);
    namedParameters.validate();
    assert(namedParameters.instance == "F-n140-k5-s5.evrp");
    assert(namedParameters.seed == 17);
    assert(!namedParameters.enableMultithreading);
    assert(!namedParameters.enableLogging);
    assert(namedParameters.popSize == 120);
    assert(std::fabs(namedParameters.mutationProb - 0.25) <= 1e-12);
    assert(std::fabs(namedParameters.mutationIndProb - 0.005) <= 1e-12);
    assert(namedParameters.tournamentSize == 3);
    assert(namedParameters.localSearchIntensity == LocalSearchIntensity::Medium);
    assert(namedParameters.localSearchPolicy == LocalSearchPolicy::Static);
    assert(std::fabs(namedParameters.parentPoolRatio - 0.15) <= 1e-12);
    assert(std::fabs(namedParameters.qualityRatio - 0.4) <= 1e-12);
    assert(std::fabs(namedParameters.verifiedUpperRatio - 0.10) <= 1e-12);
    assert(std::fabs(namedParameters.pureImmigrantRatio - 0.15) <= 1e-12);
    assert(std::fabs(namedParameters.gamma - 1.05) <= 1e-12);
    assert(std::fabs(namedParameters.eliteBudgetRatio - 0.20) <= 1e-12);
    assert(
        namedParameters.localSearchDepthProfile
        == LocalSearchDepthProfile::D6);
    assert(std::fabs(
        namedParameters.localSearchCostPenalty - 0.05) <= 1e-12);

    std::vector<std::string> legacyArguments = {
        "build/command_line_test",
        "C101-5.evrp",
        "1",
        "0",
        "weak",
    };
    CommandLine legacyCommandLine = make_command_line(legacyArguments);
    Parameters legacyParameters;
    legacyCommandLine.parse_parameters(legacyParameters);
    assert(legacyParameters.instance == "C101-5.evrp");
    assert(legacyParameters.enableLogging);
    assert(!legacyParameters.enableMultithreading);
    assert(legacyParameters.localSearchIntensity == LocalSearchIntensity::Weak);

    std::vector<std::string> skipArguments = {
        "build/command_line_test",
        "-ls", "skip",
    };
    CommandLine skipCommandLine = make_command_line(skipArguments);
    Parameters skipParameters;
    skipCommandLine.parse_parameters(skipParameters);
    assert(skipParameters.localSearchIntensity == LocalSearchIntensity::Skip);

    std::vector<std::string> onlineArguments = {
        "build/command_line_test",
        "-ls_policy", "online",
    };
    CommandLine onlineCommandLine = make_command_line(
        onlineArguments);
    Parameters onlineParameters;
    onlineCommandLine.parse_parameters(onlineParameters);
    onlineParameters.validate();
    assert(
        onlineParameters.localSearchPolicy
        == LocalSearchPolicy::OnlineIndividual);

    std::vector<std::string> boundedOnlineArguments = {
        "build/command_line_test",
        "-ls", "bounded_strong",
        "-ls_policy", "online",
    };
    CommandLine boundedOnlineCommandLine = make_command_line(
        boundedOnlineArguments);
    Parameters boundedOnlineParameters;
    boundedOnlineCommandLine.parse_parameters(
        boundedOnlineParameters);
    boundedOnlineParameters.validate();
    assert(
        boundedOnlineParameters.localSearchIntensity
        == LocalSearchIntensity::BoundedStrong);

    std::vector<std::string> matchedRandomArguments = {
        "build/command_line_test",
        "-ls", "bounded_strong",
        "-ls_policy", "matched-random",
    };
    CommandLine matchedRandomCommandLine = make_command_line(
        matchedRandomArguments);
    Parameters matchedRandomParameters;
    matchedRandomCommandLine.parse_parameters(
        matchedRandomParameters);
    matchedRandomParameters.validate();
    assert(
        matchedRandomParameters.localSearchPolicy
        == LocalSearchPolicy::MatchedRandom);

    std::vector<std::string> nonContextualArguments = {
        "build/command_line_test",
        "-ls", "bounded_strong",
        "-ls_policy", "non_contextual",
    };
    CommandLine nonContextualCommandLine = make_command_line(
        nonContextualArguments);
    Parameters nonContextualParameters;
    nonContextualCommandLine.parse_parameters(
        nonContextualParameters);
    nonContextualParameters.validate();
    assert(
        nonContextualParameters.localSearchPolicy
        == LocalSearchPolicy::OnlineNonContextual);

    Parameters invalidMixedIntensity;
    invalidMixedIntensity.localSearchPolicy =
        LocalSearchPolicy::RandomMixed;
    invalidMixedIntensity.localSearchIntensity =
        LocalSearchIntensity::Medium;
    assert(validation_fails(
        invalidMixedIntensity,
        "allocated ls_policy requires"));

    Parameters invalidMix;
    invalidMix.verifiedUpperRatio = 0.11;
    invalidMix.pureImmigrantRatio = 0.15;
    assert(validation_fails(invalidMix, "must not exceed 0.25"));

    Parameters undersizedPool;
    undersizedPool.popSize = 40;
    undersizedPool.parentPoolRatio = 0.10;
    assert(validation_fails(undersizedPool, "must be at least 5"));

    Parameters invalidQualityRatio;
    invalidQualityRatio.qualityRatio = 1.0;
    assert(validation_fails(invalidQualityRatio, "strictly between 0 and 1"));

    Parameters invalidEliteRatio;
    invalidEliteRatio.eliteBudgetRatio = 1.01;
    assert(validation_fails(invalidEliteRatio, "elite_rho"));

    Parameters invalidCostPenalty;
    invalidCostPenalty.localSearchCostPenalty = -0.01;
    assert(validation_fails(invalidCostPenalty, "ls_cost_penalty"));

    std::vector<std::string> invalidDepthArguments = {
        "build/command_line_test",
        "-ls_depth", "deep",
    };
    bool invalidDepthRejected = false;
    try {
        CommandLine invalidDepthCommandLine = make_command_line(
            invalidDepthArguments);
        Parameters invalidDepthParameters;
        invalidDepthCommandLine.parse_parameters(
            invalidDepthParameters);
    } catch (const std::invalid_argument& error) {
        invalidDepthRejected =
            std::string(error.what()).find("ls_depth")
            != std::string::npos;
    }
    assert(invalidDepthRejected);

    std::vector<std::string> invalidBooleanArguments = {
        "build/command_line_test",
        "-mth", "sometimes",
    };
    try {
        CommandLine invalidBooleanCommandLine = make_command_line(
            invalidBooleanArguments);
        Parameters invalidBooleanParameters;
        invalidBooleanCommandLine.parse_parameters(invalidBooleanParameters);
        assert(false);
    } catch (const std::invalid_argument&) {
    }

    return 0;
}
