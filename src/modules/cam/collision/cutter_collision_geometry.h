#pragma once

#include <TopoDS_Shape.hxx>

class QString;
class CamConfig;

namespace lcnc::cam {

/// Loads and normalizes the configured nozzle model, or builds the configured
/// analytic fallback. The returned proxy is expressed in cutter-local space.
TopoDS_Shape buildCutterCollisionProxy(const CamConfig& config, QString* errorMessage);

} // namespace lcnc::cam
