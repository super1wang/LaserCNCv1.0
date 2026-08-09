#pragma once

#include "core/kinematics/machine_topology.h"

#include <QWidget>
#include <QCheckBox>

class LaserToolpath;
class QDoubleSpinBox;
class QComboBox;
class QTableWidget;
class QPushButton;
class QGroupBox;
class QLabel;

/**
 * @brief Right-panel widget for laser toolpath parameters.
 *
 * Sections:
 *  1. 参数   — Explicit global/current-contour scope, lead-in length and deflection.
 *  2. 面分类 — Smooth angle threshold and classification mode.
 *  3. 操作   — Global and manual pipeline-stage execution.
 * Machine coordinates are hosted by a separate right-panel tab.
 */
class WidgetToolpathPanel : public QWidget
{
    Q_OBJECT
public:
    enum class ParameterScope { Global, CurrentContour };
    explicit WidgetToolpathPanel(QWidget* parent = nullptr);

    /// Bind the current toolpath for parameter display and coordinate lookup.
    void setToolpath(LaserToolpath* tp);

    void setLeadInLength(double mm);
    void setDiscretizationInterval(double mm);
    void setSmoothAngle(double deg);
    void setExtractionStrategy(int strategy);
    void setShowNormals(bool on);
    void setNormalSampleStep(double mm);
    void setMachiningModes(const QList<lcnc::MachiningMode>& modes,
                           lcnc::MachiningMode currentMode);
    void setMachineAxisLayout(const lcnc::MachineAxisLayout& layout);
    void setMachineSetupEditingEnabled(bool enabled);

    /// Show machine coordinates for contour at given index in the table.
    void showContourCoordinates(int contourIndex);
    QWidget* machineCoordinatesPage() const;
    void setActiveContour(int contourIndex);
    void refreshParameterEditors();
    ParameterScope parameterScope() const;

    double leadInLength()  const;
    double discretizationInterval() const;
    double smoothAngle()   const;
    int    extractionStrategy() const;

signals:
    void generateRequested();
    void separateFacesRequested();
    void pickMachiningFacesRequested();
    void applyMachiningFacesRequested();
    void extractContoursRequested();
    void discretizePointsRequested();
    void buildToolpathRequested();
    void solveMachinePathRequested();
    void leadInLengthChanged(double mm);
    void discretizationIntervalChanged(double mm);
    void smoothAngleChanged(double deg);
    void extractionStrategyChanged(int strategy);
    void parameterScopeChanged(bool currentContour);
    void machiningModeChanged(lcnc::MachiningMode mode);

private:
    void buildUi();

    LaserToolpath*  m_toolpath{nullptr};

    // Parameter widgets
    QDoubleSpinBox* m_spinLeadInLength{nullptr};
    QDoubleSpinBox* m_spinDeflection{nullptr};
    QComboBox*      m_comboParameterScope{nullptr};
    QLabel*         m_labelCurrentContour{nullptr};
    QGroupBox*      m_classificationGroup{nullptr};
    QComboBox*      m_comboMachiningMode{nullptr};

    // Face classification widgets
    QDoubleSpinBox* m_spinSmoothAngle{nullptr};
    QComboBox*      m_comboClassMode{nullptr};

    // Operation buttons
    QPushButton*    m_btnGenerate{nullptr};
    QPushButton*    m_btnSeparateFaces{nullptr};
    QPushButton*    m_btnPickMachiningFaces{nullptr};
    QPushButton*    m_btnApplyMachiningFaces{nullptr};
    QPushButton*    m_btnExtractContours{nullptr};
    QPushButton*    m_btnDiscretizePoints{nullptr};
    QPushButton*    m_btnBuildToolpath{nullptr};
    QPushButton*    m_btnSolveMachinePath{nullptr};


    // 法线显示参数
    QCheckBox*      m_checkShowNormals{nullptr};
    QDoubleSpinBox* m_spinNormalStep{nullptr};

    // Coordinate table
    QWidget*         m_machineCoordinatesPage{nullptr};
    QTableWidget*   m_coordTable{nullptr};
    int             m_activeContourIndex{-1};
    lcnc::MachineAxisLayout m_machineAxisLayout;

signals:
    void showNormalsToggled(bool on);
    void normalSampleStepChanged(double mm);
};
