#pragma once

#include "core/document/lcnc_document.h"   // DocumentId
#include "core/kernel/i_service.h"

#include <QString>

class QObject;

namespace lcnc {

/**
 * @brief CAD 模块对外门面接口（Phase 7）。
 *
 * 目的：将 UI / 命令对 CAD 子系统的依赖收窄为一组语义接口，避免直接持有
 * @c CadModule* 并 #include 模块内部头。
 *
 * 本接口继承 @ref IService。为避免多重继承中 IService 不明确，CadModule
 * 不再直接继承 IService，而是仅通过本接口带入 IService 身份。
 *
 * 当前接口仅覆盖了 UI 频繁触发的少量动作，未来按需扩充；CadModule 在
 * init() 时通过 ServiceRegistry 同时注册 `ICadFacade` 与具体类型。
 */
class ICadFacade : public IService
{
public:
    ~ICadFacade() override = default;

    /// 用于让调用方挂接信号槽（CadModule 是 QObject）。
    virtual QObject* asQObject() = 0;

    /// 切换 3D 视图至工件文档。@p id 缺省时使用当前活动文档。
    virtual void requestWorkpieceView(DocumentId id = kInvalidDocumentId) = 0;

    /// 当前活动工件文档 ID（无活动时返回 @c kInvalidDocumentId）。
    virtual DocumentId activeDocumentId() const = 0;
};

} // namespace lcnc
