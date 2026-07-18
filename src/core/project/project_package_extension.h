#pragma once

#include <QString>

#include <functional>

namespace lcnc {

class LcncProjectManifest;

/// Extension hooks run only while the project package staging directory exists.
/// Core owns the transaction; modules may provide data without owning package IO.
struct ProjectPackageExtension
{
    std::function<bool(const QString& stagingDirectory,
                       const LcncProjectManifest& manifest,
                       QString* errorMessage)> write;
    std::function<bool(const QString& stagingDirectory,
                       const LcncProjectManifest& manifest,
                       QString* errorMessage)> read;
};

} // namespace lcnc
