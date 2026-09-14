#pragma once

namespace lcnc {
class IKernel;
}

class CamModule;

namespace lcnc::cam {

/// Registers the OCC-free service adapters backed by the CAM facade.
///
/// The adapters own their immutable cross-thread caches and intentionally live
/// outside CamModule: the module coordinates lifecycle, while this boundary
/// translates it into the narrow contracts consumed by Process, Simulation and
/// the project explorer.
void registerCamServiceAdapters(lcnc::IKernel& kernel, CamModule& module);

} // namespace lcnc::cam
