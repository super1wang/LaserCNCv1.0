#pragma once

namespace lcnc {

/**
 * @brief Section switches used when writing a LaserCNC project package.
 */
struct ProjectSaveOptions {
    bool includeWorkpieceModel{true};
    bool includeMachineModel{false};
    bool includeParameters{true};
    bool includeCamData{true};
    bool includeCamCache{false};
};

} // namespace lcnc