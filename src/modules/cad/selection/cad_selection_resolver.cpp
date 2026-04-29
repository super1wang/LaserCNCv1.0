#include "modules/cad/selection/cad_selection_resolver.h"

namespace lcnc::cad::selection {
namespace {

int parseIntBetween(const QString& text, const QString& prefix, const QString& suffix = QString())
{
    if (!text.startsWith(prefix))
        return 0;
    QString value = text.mid(prefix.size());
    if (!suffix.isEmpty() && value.endsWith(suffix))
        value.chop(suffix.size());
    bool ok = false;
    const int parsed = value.toInt(&ok);
    return ok ? parsed : 0;
}

bool parseActiveHandleKey(const QString& text, int* elementId, int* handleIndex)
{
    const QString prefix = QStringLiteral("__sketch_active_handle_");
    if (!text.startsWith(prefix))
        return false;

    QString rest = text.mid(prefix.size());
    if (rest.endsWith(QStringLiteral("__")))
        rest.chop(2);

    const int separator = rest.indexOf(QLatin1Char('_'));
    if (separator <= 0 || separator + 1 >= rest.size())
        return false;

    bool elementOk = false;
    bool handleOk = false;
    const int parsedElementId = rest.left(separator).toInt(&elementOk);
    const int parsedHandleIndex = rest.mid(separator + 1).toInt(&handleOk);
    if (!elementOk || !handleOk)
        return false;

    if (elementId)
        *elementId = parsedElementId;
    if (handleIndex)
        *handleIndex = parsedHandleIndex;
    return true;
}

} // namespace

CadSelectionContext CadSelectionResolver::fromShapeEntries(DocumentId docId,
                                                           const QStringList& entries,
                                                           bool hasDocument,
                                                           bool sketchEditing,
                                                           bool hasSelectedSketch,
                                                           int selectedSketchId)
{
    CadSelectionContext context;
    context.docId = docId;
    context.hasDocument = hasDocument;
    context.sketchEditing = sketchEditing;
    context.hasSelectedSketch = hasSelectedSketch;
    context.selectedSketchId = selectedSketchId;
    context.selectedShapeCount = entries.size();
    for (const QString& entry : entries) {
        if (entry.isEmpty())
            continue;
        CadSelectionItem item;
        item.docId = docId;
        item.domain = CadSelectionDomain::DocumentShape;
        item.entry = entry;
        context.items.append(item);
    }
    return context;
}

CadSelectionContext CadSelectionResolver::fromDocumentTreeNode(DocumentId docId,
                                                              const QString& nodeKey,
                                                              const QString& entry,
                                                              const QStringList& leafEntries,
                                                              bool hasDocument,
                                                              bool sketchEditing)
{
    if (isFinishedSketchNode(nodeKey)) {
        CadSelectionContext context;
        context.docId = docId;
        context.hasDocument = hasDocument;
        context.sketchEditing = sketchEditing;
        context.hasSelectedSketch = true;
        context.selectedSketchId = sketchIdFromNodeKey(nodeKey);
        CadSelectionItem item;
        item.docId = docId;
        item.domain = CadSelectionDomain::Sketch;
        item.sketchId = context.selectedSketchId;
        context.items.append(item);
        return context;
    }

    if (nodeKey.startsWith(QStringLiteral("__sketch_finished_element_"))) {
        CadSelectionContext context;
        context.docId = docId;
        context.hasDocument = hasDocument;
        context.sketchEditing = sketchEditing;
        context.hasSelectedSketch = true;
        context.selectedSketchId = sketchIdFromNodeKey(nodeKey);
        CadSelectionItem item;
        item.docId = docId;
        item.domain = CadSelectionDomain::SketchElement;
        item.sketchId = context.selectedSketchId;
        item.sketchElementId = sketchElementIdFromNodeKey(nodeKey);
        context.items.append(item);
        return context;
    }

    if (!leafEntries.isEmpty())
        return fromShapeEntries(docId, leafEntries, hasDocument, sketchEditing, false, 0);

    if (!entry.isEmpty())
        return fromShapeEntries(docId, QStringList{entry}, hasDocument, sketchEditing, false, 0);

    CadSelectionContext context;
    context.docId = docId;
    context.hasDocument = hasDocument;
    context.sketchEditing = sketchEditing;
    return context;
}

CadSelectionContext CadSelectionResolver::fromOverlayKey(DocumentId docId,
                                                        const QString& overlayKey,
                                                        bool hasDocument,
                                                        bool sketchEditing)
{
    int handleElementId = 0;
    int handleIndex = -1;
    if (parseActiveHandleKey(overlayKey, &handleElementId, &handleIndex)) {
        CadSelectionContext context;
        context.docId = docId;
        context.hasDocument = hasDocument;
        context.sketchEditing = sketchEditing;
        CadSelectionItem item;
        item.docId = docId;
        item.domain = CadSelectionDomain::SketchElement;
        item.sketchElementId = handleElementId;
        item.sketchHandleIndex = handleIndex;
        context.items.append(item);
        return context;
    }

    if (overlayKey.startsWith(QStringLiteral("__sketch_active_element_"))) {
        CadSelectionContext context;
        context.docId = docId;
        context.hasDocument = hasDocument;
        context.sketchEditing = sketchEditing;
        CadSelectionItem item;
        item.docId = docId;
        item.domain = CadSelectionDomain::SketchElement;
        item.sketchElementId = activeSketchElementIdFromNodeKey(overlayKey);
        context.items.append(item);
        return context;
    }

    return fromDocumentTreeNode(docId,
                                overlayKey,
                                QString(),
                                {},
                                hasDocument,
                                sketchEditing);
}

bool CadSelectionResolver::isFinishedSketchNode(const QString& nodeKey)
{
    return nodeKey.startsWith(QStringLiteral("__sketch_finished_"))
        && !nodeKey.startsWith(QStringLiteral("__sketch_finished_element_"));
}

int CadSelectionResolver::sketchIdFromNodeKey(const QString& nodeKey)
{
    if (isFinishedSketchNode(nodeKey))
        return parseIntBetween(nodeKey, QStringLiteral("__sketch_finished_"), QStringLiteral("__"));

    const QString prefix = QStringLiteral("__sketch_finished_element_");
    if (!nodeKey.startsWith(prefix))
        return 0;
    const QString rest = nodeKey.mid(prefix.size());
    const int separator = rest.indexOf(QLatin1Char('_'));
    if (separator <= 0)
        return 0;
    bool ok = false;
    const int sketchId = rest.left(separator).toInt(&ok);
    return ok ? sketchId : 0;
}

int CadSelectionResolver::sketchElementIdFromNodeKey(const QString& nodeKey)
{
    const QString prefix = QStringLiteral("__sketch_finished_element_");
    if (!nodeKey.startsWith(prefix))
        return 0;
    QString rest = nodeKey.mid(prefix.size());
    if (rest.endsWith(QStringLiteral("__")))
        rest.chop(2);
    const int separator = rest.indexOf(QLatin1Char('_'));
    if (separator <= 0 || separator + 1 >= rest.size())
        return 0;
    bool ok = false;
    const int elementId = rest.mid(separator + 1).toInt(&ok);
    return ok ? elementId : 0;
}

int CadSelectionResolver::activeSketchElementIdFromNodeKey(const QString& nodeKey)
{
    return parseIntBetween(nodeKey,
                           QStringLiteral("__sketch_active_element_"),
                           QStringLiteral("__"));
}

} // namespace lcnc::cad::selection