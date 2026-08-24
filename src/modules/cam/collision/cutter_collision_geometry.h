#pragma once

#include <TopoDS_Shape.hxx>

class QString;
class CamConfig;

namespace lcnc::cam {

/// Loads and normalizes the configured nozzle model, or builds the configured
/// analytic fallback for presentation only.  This shape must never be added
/// to collision geometry; the complete cutting head belongs to the fixed
/// machine package's Z-axis body.
TopoDS_Shape buildCutterDisplayProxy(const CamConfig& config,
                                     QString* errorMessage);

} // namespace lcnc::cam
