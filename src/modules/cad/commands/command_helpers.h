#pragma once

#include "core/command/commands_api.h"
#include "core/document/lcnc_document.h"

#include <QList>
#include <QString>
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>

class GuiDocument;
class QComboBox;
class QDoubleSpinBox;

namespace lcnc::cad::commands {

/// Lightweight CAD entity payload used by command dialogs and viewport selection helpers.
struct EntityInfo {
    TDF_Label label;
    QString name;
    TopoDS_Shape shape;
};

/// Return the document matching the current UI tab context.
LcncDocument* contextualDocument(IAppContext* ctx);

/// Return the GUI document matching the current UI tab context.
GuiDocument* contextualGuiDocument(IAppContext* ctx);

/// Collect entities of a specific document kind.
QList<EntityInfo> collectEntities(
    LcncDocument* doc,
    LcncDocument::EntityKind kind = LcncDocument::EntityKind::Workpiece);

/// Collect entities relevant to the current UI tab context.
QList<EntityInfo> collectContextualEntities(IAppContext* ctx);

/// Return true when the contextual document has at least the requested entity count.
bool hasEntities(IAppContext* ctx, int minCount = 1);

/// Commit a newly built workpiece shape and refresh command states.
void commitShape(IAppContext* ctx, const TopoDS_Shape& shape, const QString& name);

/// Build a QDoubleSpinBox with shared CAD command defaults.
QDoubleSpinBox* makeSpin(double val,
                         double lo,
                         double hi,
                         int decimals = 3,
                         const QString& suffix = QStringLiteral(" mm"));

/// Return entities from all that are currently selected in the viewport.
QList<EntityInfo> selectedEntities(IAppContext* ctx, const QList<EntityInfo>& all);

/// Find the list index of a target entity by label entry.
int entityIndex(const QList<EntityInfo>& all, const EntityInfo& target);

/// Populate an entity combobox and optionally prepend a current-selection sentinel.
bool setupEntityCombo(QComboBox* cb,
                      const QList<EntityInfo>& all,
                      const QList<EntityInfo>& sel);

/// Resolve two entity indices using viewport selection or a compact picker dialog.
bool pickTwoEntities(const QString& title,
                     const QList<EntityInfo>& entities,
                     int& idxA,
                     int& idxB,
                     const QList<EntityInfo>& sel = {});

} // namespace lcnc::cad::commands
