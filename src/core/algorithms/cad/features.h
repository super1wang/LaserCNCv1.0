#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

namespace lcnc::cad_algo {

/**
 * @brief Create a prism feature from a planar profile, face, wire, or solid section.
 */
TopoDS_Shape extrudeShape(const TopoDS_Shape& profile,
                          const gp_Dir& direction,
                          double length);

/**
 * @brief Revolve a profile around an axis by @p angleDeg degrees.
 */
TopoDS_Shape revolveShape(const TopoDS_Shape& profile,
                          const gp_Ax1& axis,
                          double angleDeg);

/**
 * @brief Sweep @p profile along @p spine to create a pipe-like feature.
 */
TopoDS_Shape sweepShape(const TopoDS_Wire& profile,
                        const TopoDS_Wire& spine);

} // namespace lcnc::cad_algo
