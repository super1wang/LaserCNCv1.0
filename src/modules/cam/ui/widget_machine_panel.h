#pragma once

#include <QWidget>
#include <QString>
#include <QMap>
#include <QStringList>

#include "modules/cam/services/cam_config.h"

class LcncDocument;
class MachineKinematics;
class QLabel;
class QComboBox;
class QFormLayout;
class QGroupBox;
class QDoubleSpinBox;
class QGridLayout;
class QLineEdit;
class QPushButton;
class QTabWidget;

/**
 * @brief Right-panel widget shown when the "准备" tab is active.
 *
 * Sections:
 *  1. 机台模型页 — preset, model loading and machine-part assignment.
 *  2. 轴系配置页 — axis calibration, origins and cutter-head alignment.
 *  3. 工件配置页 — mounting, install position and rotary-center alignment.
 */
class WidgetMachinePanel : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetMachinePanel(QWidget* parent = nullptr);

    /// Called whenever the active document changes.  Rebuilds axis spinboxes
    /// and workpiece mount combo to reflect the current document's state.
    void setDocument(LcncDocument* doc);

    /// Called from MainWindow whenever the 3D selection changes.
    void setSelectedEntries(const QStringList& entries);
    void setCalibrationPickAxis(const QString& axisName);
    void setMachineModelPath(const QString& path);
    void setMachineRenderQuality(MachineRenderQuality quality);

signals:
    void machinePresetChanged(const QString& presetName);
    void machineModelPathChanged(const QString& path);
    void machineRenderQualityChanged(MachineRenderQuality quality);
    void loadMachineRequested();
    void compressMachineRequested();
    void mountWorkpieceRequested();
    void unloadMachineRequested();
    void exportMachineRequested();
    void axisOriginChanged(const QString& axisName, double x, double y, double z);
    void calibrationFacePickRequested(const QString& targetName);
    void alignToPhysicalCenterRequested(double x, double y, double z);
    void cutterHeadModelPositionChanged(double x, double y, double z);
    void cutterHeadPhysicalPositionChanged(double x, double y, double z);
    void alignToPhysicalCutterHeadRequested();
    void workpieceInstallPositionChanged(double x, double y, double z);
    void alignWorkpieceRotationCenterRequested();

private slots:
    void onAxisOriginEditorChanged();
    void onCutterHeadModelEditorChanged();
    void onCutterHeadPhysicalEditorChanged();
    void onWorkpieceInstallPositionChanged();

private:
    void buildUi();
    void buildModelPage();
    void buildConfigPage();
    void buildWorkpiecePage();
    void refreshCalibrationSection();
    void rebuildAssignmentSection();
    void rebuildWpcSection();

    LcncDocument* m_doc{nullptr};

    QTabWidget* m_pages{nullptr};
    QWidget*    m_modelPage{nullptr};
    QWidget*    m_configPage{nullptr};
    QWidget*    m_workpiecePage{nullptr};

    // Static widgets
    QComboBox* m_comboPreset{nullptr};
    QLabel* m_lblCalibrationHint{nullptr};
    QLabel* m_lblCurrentAcCenter{nullptr};
    QLabel* m_lblPickStatus{nullptr};
    QLabel* m_lblHeadModelPoint{nullptr};
    QGroupBox* m_groupAcAxes{nullptr};
    QGroupBox* m_groupAcCenter{nullptr};
    QGroupBox* m_groupHeadAlignment{nullptr};
    QDoubleSpinBox* m_axisAySpin{nullptr};
    QDoubleSpinBox* m_axisAzSpin{nullptr};
    QDoubleSpinBox* m_axisCxSpin{nullptr};
    QDoubleSpinBox* m_targetCenterX{nullptr};
    QDoubleSpinBox* m_targetCenterY{nullptr};
    QDoubleSpinBox* m_targetCenterZ{nullptr};
    QDoubleSpinBox* m_headModelX{nullptr};
    QDoubleSpinBox* m_headModelY{nullptr};
    QDoubleSpinBox* m_headModelZ{nullptr};
    QDoubleSpinBox* m_headPhysicalX{nullptr};
    QDoubleSpinBox* m_headPhysicalY{nullptr};
    QDoubleSpinBox* m_headPhysicalZ{nullptr};
    QPushButton* m_btnPickAxisA{nullptr};
    QPushButton* m_btnPickAxisC{nullptr};
    QPushButton* m_btnPickHead{nullptr};
    QPushButton* m_btnAlignToPhysical{nullptr};
    QPushButton* m_btnAlignHeadToPhysical{nullptr};

    QLabel*    m_lblMachineName{nullptr};
    QLineEdit* m_editMachinePath{nullptr};
    QComboBox* m_comboRenderQuality{nullptr};
    QGroupBox* m_assignGroup{nullptr};
    QLabel*    m_lblAssignSelection{nullptr};
    QGridLayout* m_assignGrid{nullptr};
    QGroupBox* m_wpcGroup{nullptr};
    QLabel* m_lblWorkpieceStatus{nullptr};
    QPushButton* m_btnMountWorkpiece{nullptr};
    QGroupBox* m_installGroup{nullptr};
    QDoubleSpinBox* m_wpcInstallX{nullptr};
    QDoubleSpinBox* m_wpcInstallY{nullptr};
    QDoubleSpinBox* m_wpcInstallZ{nullptr};
    QPushButton* m_btnAlignRotationCenter{nullptr};
    QStringList m_selectedEntries;
    QString m_pendingCalibrationAxis;
};
