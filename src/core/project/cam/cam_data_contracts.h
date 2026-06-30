#pragma once

#include <cstdint>
#include <string>

namespace lcnc::cam {

using ToolpathId = std::uint64_t;
using ContourId = std::uint64_t;

struct ToolpathRevision {
    std::uint64_t value{0};

    bool isValid() const { return value != 0; }
};

enum class ToolpathDirtyFlag : std::uint32_t {
    None = 0x0,
    Contours = 0x1,
    LeadIns = 0x2,
    Normals = 0x4,
    MachineCoordinates = 0x8,
    Visibility = 0x10,
    Order = 0x20,
};

using ToolpathDirtyFlags = std::uint32_t;

constexpr ToolpathDirtyFlags dirtyFlagMask(ToolpathDirtyFlag flag)
{
    return static_cast<ToolpathDirtyFlags>(flag);
}

struct ContourRenderState {
    int contourIndex{-1};
    ContourId contourId{0};
    bool enabled{true};
    bool hasLeadIn{false};
    std::string workpieceEntry;
};

} // namespace lcnc::cam