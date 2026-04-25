#pragma once

/**
 * @file render_quality.h
 * @brief 通用渲染细分质量枚举（view 层基础类型）。
 *
 * 历史上该枚举位于 `modules/cam/settings/cam_config.h`，导致 `view/`
 * 反向 include `modules/`。Phase A1 将其上提到 view 层，cam 模块按需
 * 消费即可，从而恢复 `core/ ⟵ view/ ⟵ modules/*` 的单向依赖。
 */
namespace lcnc {

/// 机台模型在 OCC 视图中的细分质量等级（用于 BRepMesh 步进/曲率参数）。
enum class MachineRenderQuality {
    High   = 0,
    Medium = 1,
    Low    = 2,
};

} // namespace lcnc

// 兼容旧未限定名引用（cam_config / gui_document 历史代码使用未带命名空间）。
using MachineRenderQuality = lcnc::MachineRenderQuality;
