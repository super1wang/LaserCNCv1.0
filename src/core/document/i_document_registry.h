#pragma once

#include <QList>

#include "core/document/lcnc_application.h"
#include "core/kernel/i_service.h"

namespace lcnc {

/**
 * @brief 文档注册表服务接口。
 *
 * 包装现有 @ref LcncApplication 单例。模块通过本接口访问：
 *   - 工件文档列表 / 当前活动文档；
 *   - 机台工作空间文档（CAM 专用）；
 *
 * 写操作（新建 / 打开 / 关闭）通过 @c ICommandBus 提交命令，不在本
 * 接口暴露。
 */
class IDocumentRegistry : public IService
{
public:
    virtual DocumentId            activeDocumentId()    const = 0;
    virtual LcncDocument*         activeDocument()      const = 0;
    virtual LcncDocument*         documentById(DocumentId id) const = 0;
    virtual QList<LcncDocument*>  documents()           const = 0;
    virtual int                   documentCount()       const = 0;

    virtual DocumentId            machineDocumentId()   const = 0;
    virtual LcncDocument*         machineDocument()     const = 0;
    virtual QList<LcncDocument*>  workpieceDocuments()  const = 0;

};

} // namespace lcnc
