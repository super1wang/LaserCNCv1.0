#pragma once

#include <QHash>
#include <QString>
#include <QVector>
#include <QWidget>

#include "modules/cad/selection/cad_selection.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGridLayout;
class QGroupBox;
class QLabel;
class QListWidget;
class QPushButton;
class QSpinBox;
class QStackedWidget;

namespace lcnc::cad::task { struct CadToolDescriptor; }

namespace lcnc::cad::ui {

/**
 * @brief Right-side CAD workflow panel with a task home and per-command pages.
 */
class WidgetCadTaskPanel : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetCadTaskPanel(QWidget* parent = nullptr);

    /// Current feature length parameter in millimeters.
    double featureLength() const;
    /// Current feature angle parameter in degrees.
    double featureAngle() const;
    /// Current feature index: extrude=0, revolve=1.
    int featureIndex() const;
    /// Current primitive index: box=0, cylinder=1, sphere=2, cone=3, torus=4.
    int primitiveIndex() const;
    /// Current primitive X size parameter in millimeters.
    double primitiveSizeX() const;
    /// Current primitive Y size parameter in millimeters.
    double primitiveSizeY() const;
    /// Current primitive Z size or height parameter in millimeters.
    double primitiveSizeZ() const;
    /// Current primitive first radius parameter in millimeters.
    double primitiveRadius1() const;
    /// Current primitive second radius parameter in millimeters.
    double primitiveRadius2() const;
    /// Whether transient feature preview is enabled.
    bool isPreviewEnabled() const;
    /// Returns true when the panel is currently editing a feature command.
    bool isFeaturePageActive() const;
    /// Returns true when the panel is currently editing a primitive command.
    bool isPrimitivePageActive() const;
    /// Returns true when the panel is currently editing an interactive transform.
    bool isTransformPageActive() const;
    /// Current transform reference mode: model center=0, world origin=1.
    int transformReferenceMode() const;
    /// Current transform translation X in millimeters.
    double transformTranslateX() const;
    /// Current transform translation Y in millimeters.
    double transformTranslateY() const;
    /// Current transform translation Z in millimeters.
    double transformTranslateZ() const;
    /// Current transform rotation X in degrees.
    double transformRotateX() const;
    /// Current transform rotation Y in degrees.
    double transformRotateY() const;
    /// Current transform rotation Z in degrees.
    double transformRotateZ() const;

    /// Lightweight summary of a finished sketch shown on the home page.
    struct FinishedSketchEntry {
        int sketchId{0};
        QString label;
        bool visible{true};
        bool usedByFeature{false};
    };

    /// Refresh available home commands from document, selection, and sketch state.
    /// `hasSelectedSketch` controls whether feature buttons (extrude/revolve) are shown.
    void setContextState(bool hasDocument,
                         int selectedCount,
                         bool sketchEditing,
                         bool hasSelectedSketch);
    /// Refresh available home commands from a normalized CAD selection context.
    void setSelectionContext(const lcnc::cad::selection::CadSelectionContext& context);
    /// Replace the home-page "草图列表" entries (id + label + used flag).
    void setFinishedSketches(const QVector<FinishedSketchEntry>& entries,
                             int selectedSketchId);

    /// Update the sketch tool palette to reflect the active tool kind.
    void setActiveSketchTool(int toolKind);
    /// Replace the sketch element list with the supplied entries.
    /// Each entry is encoded as (id, kind, label).
    struct SketchElementEntry {
        int id{0};
        int kind{0};
        QString label;
    };
    void setSketchElements(const QVector<SketchElementEntry>& elements);

    /// Switch back to the workflow home page.
    void showHomePage();
    /// Switch to the sketch command page.
    void showSketchPage();
    /// Switch to the primitive command page and select the requested primitive.
    void showPrimitivePage(int primitiveIndex);
    /// Switch to the feature command page and select the requested feature.
    void showFeaturePage(int featureIndex);
    /// Switch to the interactive transform page.
    void showTransformPage();
    /// Apply a view-gizmo drag delta into the transform parameter widgets.
    void addTransformDragDelta(int operation, int axis, double delta);

signals:
    void commandRequested(const QString& commandId);
    void sketchCreateRequested(int planeIndex);
    void sketchExitRequested();
    void sketchCanceled();
    /// User selected a different finished sketch on the home page list.
    void sketchSelectionChanged(int sketchId);
    /// User asked to delete a finished sketch via the manager.
    void sketchDeleteRequested(int sketchId);
    /// User toggled visibility of a finished sketch on the home page list.
    void sketchVisibilityToggled(int sketchId, bool visible);
    void primitiveParametersChanged(int primitiveIndex,
                                    double sizeX,
                                    double sizeY,
                                    double sizeZ,
                                    double radius1,
                                    double radius2);
    void primitiveApplyRequested(int primitiveIndex,
                                 double sizeX,
                                 double sizeY,
                                 double sizeZ,
                                 double radius1,
                                 double radius2);
    void primitiveCanceled();
    void featureParametersChanged(int featureIndex, double length, double angleDeg);
    void featureApplyRequested(int featureIndex, double length, double angleDeg);
    void featureCanceled();
    void transformParametersChanged(double translateX,
                                    double translateY,
                                    double translateZ,
                                    double rotateX,
                                    double rotateY,
                                    double rotateZ,
                                    int referenceMode);
    void transformApplyRequested(double translateX,
                                 double translateY,
                                 double translateZ,
                                 double rotateX,
                                 double rotateY,
                                 double rotateZ,
                                 int referenceMode);
    void transformCanceled();
    void previewToggled(bool enabled);
    /// User changed the active sketch drawing tool.
    void sketchToolChanged(int toolKind);
    /// User asked to commit a new sketch element with the supplied parameters.
    void sketchElementAddRequested(int toolKind, QVector<double> params);
    /// User asked to remove a sketch element by id.
    void sketchElementRemoveRequested(int elementId);

private:
    void buildUi();
    void buildHomePage();
    void buildSketchPage();
    void buildPrimitivePage();
    void buildFeaturePage();
    void buildTransformPage();
    void updateHomeAvailability();
    void updatePrimitiveParameterVisibility();
    void emitPrimitiveParametersChanged();
    void emitFeatureParametersChanged();
    void emitTransformParametersChanged();
    void rebuildSketchToolForm(int toolKind);
    QVector<double> currentSketchToolParams() const;
    QPushButton* makeToolButton(const lcnc::cad::task::CadToolDescriptor& tool,
                                QWidget* parent);
    void addToolButtons(QGridLayout* layout,
                        const QVector<lcnc::cad::task::CadToolDescriptor>& tools,
                        QWidget* parent,
                        int columns = 2);

    QStackedWidget* m_stack{nullptr};
    QWidget* m_pageHome{nullptr};
    QWidget* m_pageSketch{nullptr};
    QWidget* m_pagePrimitive{nullptr};
    QWidget* m_pageFeature{nullptr};
    QWidget* m_pageTransform{nullptr};
    QGroupBox* m_groupDocument{nullptr};
    QGroupBox* m_groupBase{nullptr};
    QGroupBox* m_groupProfile{nullptr};
    QGroupBox* m_groupSketches{nullptr};
    QGroupBox* m_groupSelection{nullptr};
    QGroupBox* m_groupBoolean{nullptr};
    QGroupBox* m_groupMeasure{nullptr};
    QGroupBox* m_groupDelete{nullptr};
    QGroupBox* m_groupFaceTools{nullptr};
    QGroupBox* m_groupEdgeTools{nullptr};
    QComboBox* m_comboPlane{nullptr};
    QComboBox* m_comboPrimitive{nullptr};
    QComboBox* m_comboFeature{nullptr};
    QComboBox* m_comboTransformReference{nullptr};
    QDoubleSpinBox* m_spinPrimitiveSizeX{nullptr};
    QDoubleSpinBox* m_spinPrimitiveSizeY{nullptr};
    QDoubleSpinBox* m_spinPrimitiveSizeZ{nullptr};
    QDoubleSpinBox* m_spinPrimitiveRadius1{nullptr};
    QDoubleSpinBox* m_spinPrimitiveRadius2{nullptr};
    QDoubleSpinBox* m_spinFeatureLength{nullptr};
    QDoubleSpinBox* m_spinFeatureAngle{nullptr};
    QDoubleSpinBox* m_spinTransformTranslateX{nullptr};
    QDoubleSpinBox* m_spinTransformTranslateY{nullptr};
    QDoubleSpinBox* m_spinTransformTranslateZ{nullptr};
    QDoubleSpinBox* m_spinTransformRotateX{nullptr};
    QDoubleSpinBox* m_spinTransformRotateY{nullptr};
    QDoubleSpinBox* m_spinTransformRotateZ{nullptr};
    QListWidget* m_listFinishedSketches{nullptr};
    QPushButton* m_btnDeleteFinishedSketch{nullptr};
    QPushButton* m_btnToggleSketchVisible{nullptr};
    QCheckBox* m_checkPreview{nullptr};
    QCheckBox* m_checkPrimitivePreview{nullptr};
    QCheckBox* m_checkTransformPreview{nullptr};
    QLabel* m_labelPrimitiveSizeX{nullptr};
    QLabel* m_labelPrimitiveSizeY{nullptr};
    QLabel* m_labelPrimitiveSizeZ{nullptr};
    QLabel* m_labelPrimitiveRadius1{nullptr};
    QLabel* m_labelPrimitiveRadius2{nullptr};
    QPushButton* m_btnOpenSketchPage{nullptr};
    QPushButton* m_btnStartSketch{nullptr};
    QPushButton* m_btnExitSketch{nullptr};
    QPushButton* m_btnCancelSketch{nullptr};
    QGroupBox* m_groupSketchTool{nullptr};
    QComboBox* m_comboSketchTool{nullptr};
    QWidget* m_sketchToolForm{nullptr};
    QListWidget* m_listSketchElements{nullptr};
    QPushButton* m_btnAddSketchElement{nullptr};
    QPushButton* m_btnRemoveSketchElement{nullptr};
    QVector<QDoubleSpinBox*> m_sketchToolSpins;
    QVector<QSpinBox*> m_sketchToolIntSpins;
    QPushButton* m_btnApplyPrimitive{nullptr};
    QPushButton* m_btnCancelPrimitive{nullptr};
    QPushButton* m_btnExtrudeFeature{nullptr};
    QPushButton* m_btnRevolveFeature{nullptr};
    QPushButton* m_btnApplyFeature{nullptr};
    QPushButton* m_btnCancelFeature{nullptr};
    QPushButton* m_btnApplyTransform{nullptr};
    QPushButton* m_btnCancelTransform{nullptr};
    bool m_hasDocument{false};
    int m_selectedCount{0};
    bool m_sketchEditing{false};
    bool m_hasSelectedSketch{false};
    int m_selectedSketchId{0};
    lcnc::cad::selection::CadSelectionContext m_selectionContext;
    QHash<QString, QPushButton*> m_toolButtons;
};

} // namespace lcnc::cad::ui