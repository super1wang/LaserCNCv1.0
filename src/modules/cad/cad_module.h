#pragma once

#include <QObject>
#include <QList>
#include <QSet>
#include <QStringList>
#include <QVector>
#include <QVariantMap>
#include <memory>

#include "core/task/module_task_scope.h"
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>

#include "core/document/lcnc_document.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "core/project/project_types.h"
#include "core/task/task_manager.h"
#include "modules/cad/i_cad_facade.h"
#include "modules/cad/selection/cad_selection.h"

class GuiDocument;
class GuiApplication;
class TaskManager;
class gp_Vec;
class gp_Ax1;
namespace lcnc::cad { class CadDocumentRegistry; class CadDocumentIoService; class CadModelingSession; class SketchManager; }
namespace lcnc::cad::task { class CadCommandDispatcher; }

/**
 * @brief CAD module singleton — manages documents, file I/O, and modeling operations.
 *
 * Responsible for:
 *  - Document lifecycle (new / open / save / close / import / export)
 // 中文翻译：文档
 *  - Managing the "Documentation" (Document) tab page
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
    /**
     * @brief Parameter bundle for TaskPanel primitive creation.
     *
     * Mapping by primitive index:
     *  - box: sizeX, sizeY, sizeZ
     *  - cylinder: radius1, sizeZ
     *  - sphere: radius1
     *  - cone: radius1, radius2, sizeZ
     *  - torus: radius1, radius2
     */
    struct PrimitiveParameters {
        double sizeX{100.0};
        double sizeY{100.0};
        double sizeZ{50.0};
        double radius1{50.0};
        double radius2{15.0};
    };

    /**
     * @brief Parameter bundle for interactive transform preview and apply.
     */
    struct TransformParameters {
        double translateX{0.0};
        double translateY{0.0};
        double translateZ{0.0};
        double rotateX{0.0};
        double rotateY{0.0};
        double rotateZ{0.0};
        int referenceMode{0}; ///< 0=model center, 1=world origin.
    };

    /**
     * @brief Lightweight snapshot of a sketch element for tree/list rendering.
     */
    struct SketchElementSnapshot {
        int id{0};
        int kind{0};
        QString label;
    };

    /**
     * @brief Lightweight snapshot of a finished sketch record.
     */
    struct FinishedSketchSnapshot {
        int sketchId{0};
        QString name;
        bool visible{true};
        bool usedByFeature{false};
        QList<SketchElementSnapshot> elements;
    };

    /**
     * @brief View-neutral sketch overlay snapshot used by MainWindow to feed the view renderer.
     */
    struct SketchOverlaySnapshot {
        QString key;
        int sketchId{0};
        int elementId{0};
        int kind{0};
        int plane{0};
        QVector<double> params;
        bool visible{true};
        bool selected{false};
        bool activeSession{false};
        bool usedByFeature{false};
        bool draggable{false};
    };

    // ── IModule ───────────────────────────────────────────────────
    /// 模块元信息：id="cad"，无依赖；CAM/Process 依赖本模块。
    lcnc::ModuleInfo info() const override;
    /// 注册自身为 IService 并建立项目域信号转发。失败返回 false。
    bool init(lcnc::IKernel& kernel) override;
    /// 启动阶段：当前为占位（init 已完成所有连接）。
    bool start() override;
    /// 反向释放：注销服务。已注册的 Qt 连接由 QObject 在析构时自动断开。
    void stop() override;

    // ── Project / Workpiece Management ──────────────────────────────────
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

    // ── Project Domain Access ────────────────────────────────────────────
    DocumentId    workpieceDocumentId() const override;
    LcncDocument* workpieceDocument() const;
    GuiDocument*  activeGuiDocument() const;
    LcncDocument* domainDocumentById(DocumentId id) const;
    void          requestWorkpieceView(DocumentId id = kInvalidDocumentId) override;

    /// ICadFacade：用于让调用方挂接 CadModule 的 Qt 信号。
    QObject* asQObject() override { return this; }

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(DocumentId docId, const QString& entry, bool visible);
    void setEntriesVisible(DocumentId docId, const QStringList& entries, bool visible);
    void setSelectedEntries(DocumentId docId, const QStringList& entries);
    QStringList selectedEntries(DocumentId docId) const;
    void syncSelectionFromView(DocumentId docId = kInvalidDocumentId);
    lcnc::cad::selection::CadSelectionContext selectionContext(
        DocumentId docId = kInvalidDocumentId) const;
    void setSelectionContext(const lcnc::cad::selection::CadSelectionContext& context);

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

    /// Request the UI to enter a primitive creation tool from Ribbon or Task home.
    void requestPrimitiveTool(int primitiveIndex);
    /// Build a transient primitive preview shape without modifying any document.
    bool buildPrimitivePreview(int primitiveIndex,
                               const PrimitiveParameters& params,
                               TopoDS_Shape* outShape,
                               QString* errMsg = nullptr);
    /// Create a primitive and commit it to the active or newly created document.
    bool createPrimitive(int primitiveIndex, const PrimitiveParameters& params);
    /// Build a preview through the CAD tool dispatcher.
    bool previewTool(const QString& toolId,
                     const QVariantMap& params,
                     TopoDS_Shape* outShape,
                     QString* errMsg = nullptr);
    /// Execute a parameterized CAD tool through the dispatcher.
    bool executeTool(const QString& toolId,
                     const QVariantMap& params,
                     QString* errMsg = nullptr);
    /// Build a transient transform preview for the selected shapes.
    bool buildTransformPreview(const TransformParameters& params,
                               TopoDS_Shape* outShape,
                               double* refX = nullptr,
                               double* refY = nullptr,
                               double* refZ = nullptr,
                               QString* errMsg = nullptr) const;
    /// Apply an interactive transform to the selected shapes.
    bool applyTransform(const TransformParameters& params,
                        QString* errMsg = nullptr);

    // ── Sketch + Feature Modeling ───────────────────────────────────────
    /// Start a lightweight sketch session on the requested plane index.
    bool beginSketch(int planeIndex);
    /// Finish the active sketch and push it as a SketchRecord into the manager.
    bool finishSketch();
    /// Apply a feature to the currently selected finished sketch and commit it.
    bool applyFeature(int featureIndex, double length, double angleDeg);
    /// Build a transient feature preview shape without modifying any document.
    bool buildFeaturePreview(int featureIndex,
                             double length,
                             double angleDeg,
                             TopoDS_Shape* outShape,
                             QString* errMsg = nullptr);
    /// Clear the current transient modeling operation without document mutation.
    void cancelModelingOperation();
    /// Returns true while the module has an editable sketch session.
    bool isSketchEditing() const;
    /// Returns true when a finished sketch is currently selected and ready for features.
    bool hasSelectedSketch() const;
    /// Currently selected finished sketch id (0 if none).
    int selectedSketchId() const;
    /// Select a finished sketch by id (0 to clear). Emits sketchSelectionChanged.
    void setSelectedSketchId(int sketchId);
    /// Snapshot of finished sketches for the active document.
    QList<FinishedSketchSnapshot> finishedSketchSnapshots(DocumentId docId = kInvalidDocumentId) const;
    /// Set visibility flag of a finished sketch (manager-side bookkeeping; view layer pending).
    bool setSketchVisible(DocumentId docId, int sketchId, bool visible);
    /// Delete a finished sketch by id.
    bool deleteSketch(DocumentId docId, int sketchId);

    /// Set the currently active sketch tool (Ribbon command or TaskPanel palette).
    void setSketchTool(int toolKind);
    /// Read the currently active sketch tool kind.
    int sketchTool() const;
    /// Append a new sketch element to the active session. Returns the assigned id or -1.
    int addSketchElement(int toolKind, const QVector<double>& params, QString* errMsg = nullptr);
    /// Remove a previously added sketch element by id.
    bool removeSketchElement(int elementId);
    /// Move a sketch element in the active sketch session.
    bool moveSketchElement(int elementId, double deltaX, double deltaY, QString* errMsg = nullptr);
    /// Move a specific sketch element handle in the active sketch session.
    bool moveSketchElementHandle(int elementId,
                                 int handleIndex,
                                 double deltaX,
                                 double deltaY,
                                 QString* errMsg = nullptr);
    /// Snapshot of current sketch elements for tree/list rendering.
    QList<SketchElementSnapshot> sketchElementSnapshots() const;
    /// Snapshot of sketch elements for view-layer overlay rendering.
    QList<SketchOverlaySnapshot> sketchOverlaySnapshots(DocumentId docId = kInvalidDocumentId) const;

    // ── Undo / Redo ─────────────────────────────────────────────────────
    bool canUndo(DocumentId docId) const;
    bool canRedo(DocumentId docId) const;
    void undo(DocumentId docId);
    void redo(DocumentId docId);

signals:
    /// Emitted whenever the document list changes (add/remove).
    void documentListChanged();
    /// Emitted when the workpiece section tree should be rebuilt.
    void workpieceStructureChanged();
    /// Emitted when the UI should attach the requested workpiece workspace view.
    void workpieceViewRequested(DocumentId id);
    /// Emitted when a document's content is modified.
    void documentModified(DocumentId id);
    /// Emitted when a module-level operation fails and should be surfaced by the UI.
    void operationFailed(const QString& title, const QString& message);
    /// Emitted when a workpiece selection changes via module coordination.
    void selectionChanged(DocumentId id, const QStringList& entries);
    /// Emitted when a command requests activation of a primitive TaskPanel tool.
    void primitiveToolRequested(int primitiveIndex);
    /// Emitted when the active sketch tool kind changes.
    void sketchToolChanged(int toolKind);
    /// Emitted when the sketch element list changes (add/remove/clear).
    void sketchElementsChanged();
    /// Emitted when finished sketch records change for a document.
    void finishedSketchesChanged(DocumentId docId);
    /// Emitted when the selected finished sketch changes.
    void sketchSelectionChanged(int sketchId);

private:
    void refreshDisplay(DocumentId docId);
    bool cancelOwnedTasks(int timeoutMs);

    /// 标记 init() 是否已成功执行（避免重复注册）。
    bool m_initialized{false};
    int m_selectedSketchId{0};
    DocumentId m_selectedSketchDocId{kInvalidDocumentId};
    std::unique_ptr<lcnc::cad::CadModelingSession> m_modelingSession;
    std::unique_ptr<lcnc::cad::CadDocumentRegistry> m_documentRegistry;
    std::unique_ptr<lcnc::cad::CadDocumentIoService> m_documentIoService;
    std::unique_ptr<lcnc::cad::task::CadCommandDispatcher> m_commandDispatcher;
    lcnc::ModuleTaskScope m_taskScope;
};
