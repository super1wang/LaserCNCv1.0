#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"

#include <QFileInfo>

// OCC XCAF
#include <XCAFApp_Application.hxx>
#include <BinXCAFDrivers.hxx>
#include <XmlXCAFDrivers.hxx>

// STEP reader/writer
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <XCAFDoc_ShapeTool.hxx>

// IGES reader
#include <IGESControl_Reader.hxx>

// STL
#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Face.hxx>

// ── Singleton accessor ────────────────────────────────────────────────────────
//   Lifecycle owned by lcnc::Kernel — see kernel.cpp::registerCoreServices.
//   Use lcnc::Kernel::current().app() instead of any LcncApplication::instance().
LcncApplication* LcncApplication::s_instance = nullptr;

LcncApplication::LcncApplication(QObject* parent)
    : QObject(parent)
{
    Q_ASSERT_X(!s_instance, "LcncApplication",
               "second LcncApplication instance — must be Kernel-owned only");
    s_instance = this;

    // Register XDE format drivers so we can open/save XCAF documents
    Handle(XCAFApp_Application) occApp = XCAFApp_Application::GetApplication();
    BinXCAFDrivers::DefineFormat(occApp);
    XmlXCAFDrivers::DefineFormat(occApp);
}

LcncApplication::~LcncApplication()
{
    qDeleteAll(m_documents);
    if (s_instance == this) s_instance = nullptr;
}

// ── Document lifecycle ─────────────────────────────────────────────────────────
LcncDocument* LcncApplication::newDocument(const QString& name)
{
    const int id = m_nextId++;
    const QString docName = name.isEmpty() ? QStringLiteral("Document %1").arg(id + 1) : name;

    auto* doc = new LcncDocument(id, docName);
    m_documents.append(doc);
    emit documentAdded(id);
    setActiveDocument(id);
    return doc;
}

LcncDocument* LcncApplication::openDocument(const QString& filePath, QString* errorMsg)
{
    QFileInfo fi(filePath);
    if (!fi.exists()) {
        if (errorMsg) *errorMsg = QStringLiteral("文件不存在: %1").arg(filePath);
        return nullptr;
    }

    LcncDocument* doc = newDocument(fi.baseName());
    doc->setFilePath(filePath);

    const QString ext = fi.suffix().toLower();
    bool ok = false;

    if (ext == "stp" || ext == "step") {
        STEPControl_Reader reader;
        if (reader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone) {
            reader.TransferRoots();
            for (int i = 1; i <= reader.NbShapes(); ++i) {
                TopoDS_Shape sh = reader.Shape(i);
                if (!sh.IsNull())
                    doc->addShapeEntity(sh, QStringLiteral("Shape_%1").arg(i));
            }
            ok = true;
        }
    }
    else if (ext == "igs" || ext == "iges") {
        IGESControl_Reader reader;
        if (reader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone) {
            reader.TransferRoots();
            for (int i = 1; i <= reader.NbShapes(); ++i) {
                TopoDS_Shape sh = reader.Shape(i);
                if (!sh.IsNull())
                    doc->addShapeEntity(sh, QStringLiteral("Shape_%1").arg(i));
            }
            ok = true;
        }
    }
    else if (ext == "stl") {
        Handle(Poly_Triangulation) mesh =
            RWStl::ReadFile(filePath.toUtf8().constData());
        if (!mesh.IsNull()) {
            BRep_Builder builder;
            TopoDS_Face face;
            builder.MakeFace(face);
            builder.UpdateFace(face, mesh);
            doc->addShapeEntity(face, fi.baseName());
            ok = true;
        }
    }
    else {
        // Try OCC native format (XCAF binary/xml) via XCAFApp directly
        ok = true; // Just create empty document for unsupported types
    }

    if (!ok) {
        if (errorMsg)
            *errorMsg = QStringLiteral("无法读取文件: %1").arg(filePath);
        closeDocument(doc->id());
        return nullptr;
    }

    emit documentModified(doc->id());

    return doc;
}

bool LcncApplication::saveDocument(DocumentId id, const QString& filePath, QString* errorMsg)
{
    LcncDocument* doc = documentById(id);
    if (!doc) {
        if (errorMsg) *errorMsg = QStringLiteral("找不到文档 ID=%1").arg(id);
        return false;
    }

    // Export all free shapes as STEP
    STEPControl_Writer writer;
    TDF_LabelSequence labels;
    doc->shapeTool()->GetFreeShapes(labels);
    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    for (int i = 1; i <= labels.Length(); ++i) {
        TopoDS_Shape sh = st->GetShape(labels.Value(i));
        if (!sh.IsNull())
            writer.Transfer(sh, STEPControl_AsIs);
    }

    const bool ok = (writer.Write(filePath.toUtf8().constData()) == IFSelect_RetDone);
    if (!ok && errorMsg)
        *errorMsg = QStringLiteral("保存失败: %1").arg(filePath);

    if (ok) {
        doc->setFilePath(filePath);
        emit documentModified(id);
    }
    return ok;
}

LcncDocument* LcncApplication::ensureMachineDocument()
{
    if (m_machineDocId != kInvalidDocumentId)
        return machineDocument();
    const int id = m_nextId++;
    auto* doc = new LcncDocument(id, QStringLiteral("机台工作区"));
    m_machineDocId = id;
    m_documents.append(doc);
    // Emit so GuiApplication creates a GuiDocument for the machine workspace,
    // but do NOT call setActiveDocument — the machine doc is never "active".
    emit documentAdded(id);
    return doc;
}

LcncDocument* LcncApplication::machineDocument() const
{
    return documentById(m_machineDocId);
}

QList<LcncDocument*> LcncApplication::workpieceDocuments() const
{
    QList<LcncDocument*> result;
    for (LcncDocument* d : m_documents)
        if (d->id() != m_machineDocId)
            result.append(d);
    return result;
}

void LcncApplication::closeDocument(DocumentId id)
{
    // Machine workspace document is permanent for the session lifetime.
    if (id == m_machineDocId) return;

    for (int i = 0; i < m_documents.size(); ++i) {
        if (m_documents[i]->id() == id) {
            LcncDocument* doc = m_documents.takeAt(i);

            if (m_activeId == id) {
                m_activeId = m_documents.isEmpty()
                                 ? kInvalidDocumentId
                                 : m_documents.last()->id();
                emit activeDocumentChanged(m_activeId);
            }

            emit documentClosed(id);
            delete doc;
            return;
        }
    }
}

void LcncApplication::notifyDocumentModified(DocumentId id)
{
    if (documentById(id) != nullptr)
        emit documentModified(id);
}

// ── Document access ────────────────────────────────────────────────────────────
LcncDocument* LcncApplication::documentById(DocumentId id) const
{
    for (LcncDocument* d : m_documents) {
        if (d->id() == id) return d;
    }
    return nullptr;
}

QList<LcncDocument*> LcncApplication::documents() const
{
    return m_documents;
}

int LcncApplication::documentCount() const
{
    return m_documents.size();
}

LcncDocument* LcncApplication::activeDocument() const
{
    return documentById(m_activeId);
}

void LcncApplication::setActiveDocument(DocumentId id)
{
    // Machine workspace is never the "active" document in the workpiece sense.
    if (id == m_machineDocId) return;
    if (m_activeId != id) {
        m_activeId = id;
        emit activeDocumentChanged(id);
    }
}
