#pragma once

#include <QObject>
#include <QList>
#include <QStringList>
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>

#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "modules/cad/i_cad_facade.h"

class GuiDocument;
class GuiApplication;
class TaskManager;
class gp_Vec;
class gp_Ax1;

/**
 * @brief CAD module singleton — manages documents, file I/O, and modeling operations.
 *
 * Responsible for:
 *  - Document lifecycle (new / open / save / close / import / export)
 *  - Managing the "文档" (Document) tab page
 *  - Providing modeling operations (create / move / rotate / delete / boolean / explode)
 *    via ShapeService delegation
 *  - Undo/Redo
 *
 * Ribbon commands should call CadModule APIs instead of implementing business logic directly.
 *
 * 微内核集成：本类同时实现 @ref lcnc::IModule（参与 Kernel 生命周期）和
 * @ref lcnc::IService（可被其他模块/UI 通过 services().getService<CadModule>()
 * 取到）。生命周期由 Kernel 拥有；旧的 @c instance() 仍可用，但已**不会**
 * 自动创建实例 —— 必须先 @c kernel.addModule(std::make_unique<CadModule>())
 * 并 @c bootstrap() 之后才能拿到非空指针。
 */
class CadModule : public QObject, public lcnc::IModule, public lcnc::ICadFacade
{
    Q_OBJECT
public:
    /// 公开构造函数：由 Kernel/main 通过 @c std::make_unique 持有；其余
    /// 代码应使用 @ref instance() 取得唯一实例，禁止再 new 第二个。
    explicit CadModule(QObject* parent = nullptr);
    struct DocumentTreeNode {
        QString nodeKey;
        QString displayName;
        QString entry;
        QStringList leafEntries;
        QList<DocumentTreeNode> children;
    };

    struct DocumentTreeDocument {
        DocumentId documentId{kInvalidDocumentId};
        QString nodeKey;
        QString displayName;
        QStringList leafEntries;
        QList<DocumentTreeNode> children;
    };

    // ── IModule ───────────────────────────────────────────────────
    /// 模块元信息：id="cad"，无依赖；CAM/Process 依赖本模块。
    lcnc::ModuleInfo info() const override;
    /// 注册自身为 IService 并建立与 LcncApplication 的信号槽。失败返回 false。
    bool init(lcnc::IKernel& kernel) override;
    /// 启动阶段：当前为占位（init 已完成所有连接）。
    bool start() override;
    /// 反向释放：注销服务。已注册的 Qt 连接由 QObject 在析构时自动断开。
    void stop() override;

    // ── Document Management (delegates to LcncApplication) ───────────────
    DocumentId  newDocument(const QString& name = QString());
    DocumentId  openDocument(const QString& filePath);
    DocumentId  importStep(const QString& filePath,
                           DocumentId targetDocId = kInvalidDocumentId);
    DocumentId  importStl(const QString& filePath,
                          DocumentId targetDocId = kInvalidDocumentId);
    bool        saveDocument(DocumentId id, const QString& path = QString());
    void        exportStep(DocumentId id, const QString& filePath);
    void        closeDocument(DocumentId id);
    DocumentId  importFile(const QString& filePath);

    // ── Active Document ──────────────────────────────────────────────────
    DocumentId    activeDocumentId() const override;
    LcncDocument* activeDocument() const;
    GuiDocument*  activeGuiDocument() const;
    LcncDocument* documentById(DocumentId id) const;
    GuiDocument*  guiDocument(DocumentId id) const;
    QList<LcncDocument*> workpieceDocuments() const;
    QList<DocumentTreeDocument> documentTreeDocuments() const;
    void          setActiveDocument(DocumentId id);
    void          requestWorkpieceView(DocumentId id = kInvalidDocumentId) override;

    /// ICadFacade：用于让调用方挂接 CadModule 的 Qt 信号。
    QObject* asQObject() override { return this; }

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(DocumentId docId, const QString& entry, bool visible);
    void setEntriesVisible(DocumentId docId, const QStringList& entries, bool visible);
    void setSelectedEntries(DocumentId docId, const QStringList& entries);
    QStringList selectedEntries(DocumentId docId) const;
    void syncSelectionFromView(DocumentId docId = kInvalidDocumentId);

    // ── Modeling Operations (delegates to ShapeService + refreshes display) ──
    /// Move a shape by translation vector. Returns true on success.
    bool moveShape(DocumentId docId, const TDF_Label& label, const gp_Vec& translation);
    bool moveShapes(DocumentId docId, const QList<TDF_Label>& labels,
                    const gp_Vec& translation);

    /// Rotate a shape around an axis by angleDeg degrees. Returns true on success.
    bool rotateShape(DocumentId docId, const TDF_Label& label,
                     const gp_Ax1& axis, double angleDeg);
    bool rotateShapes(DocumentId docId, const QList<TDF_Label>& labels,
                      const gp_Ax1& axis, double angleDeg);

    /// Delete a shape from a document by its label entry string.
    bool deleteShape(DocumentId docId, const QString& entry);
    bool deleteShapes(DocumentId docId, const QStringList& entries);

    /// Explode a compound into sub-shapes. Returns child count.
    int  explodeShape(DocumentId docId, const TDF_Label& label, int entityKind);

    /// Add a new shape to the document. Returns the new label.
    TDF_Label createShape(DocumentId docId, const TopoDS_Shape& shape,
                          const QString& name,
                          int entityKind = static_cast<int>(LcncDocument::EntityKind::Workpiece));

    // ── Undo / Redo ─────────────────────────────────────────────────────
    bool canUndo(DocumentId docId) const;
    bool canRedo(DocumentId docId) const;
    void undo(DocumentId docId);
    void redo(DocumentId docId);

signals:
    /// Emitted whenever the document list changes (add/remove).
    void documentListChanged();
    /// Emitted when the workpiece document tree should be rebuilt.
    void documentTreeChanged();
    /// Emitted when the active document switches.
    void activeDocumentChanged(DocumentId id);
    /// Emitted when the UI should attach the requested workpiece document view.
    void workpieceViewRequested(DocumentId id);
    /// Emitted when a document's content is modified.
    void documentModified(DocumentId id);
    /// Emitted when a module-level operation fails and should be surfaced by the UI.
    void operationFailed(const QString& title, const QString& message);
    /// Emitted when a workpiece document selection changes via module coordination.
    void selectionChanged(DocumentId id, const QStringList& entries);

private:
    void refreshDisplay(DocumentId docId);

    /// 标记 init() 是否已成功执行（避免重复注册）。
    bool m_initialized{false};
};
