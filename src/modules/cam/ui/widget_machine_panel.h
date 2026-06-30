#pragma once

#include <QWidget>
#include <QString>
#include <QMap>
#include <QStringList>

class LcncDocument;
class MachineKinematics;
class QLabel;
class QCheckBox;
class QGroupBox;
class QDoubleSpinBox;
class QEvent;
class QGridLayout;
class QPushButton;

/**
 * @brief Right-panel widget shown when the "准备" tab is active.
 *
 * Sections:
 *  1. 机台模型页 — preset, model loading, machine-part assignment, calibration and workpiece install position.
 */
class WidgetMachinePanel : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetMachinePanel(QWidget* parent = nullptr);

    /// 拦截 spin 控件滚轮，避免滚动页面时误改参数。
    bool eventFilter(QObject* watched, QEvent* event) override;

    /// Called whenever the active document changes.  Rebuilds axis spinboxes
    /// and workpiece mount combo to reflect the current document's state.
    void setDocument(LcncDocument* doc);

    /// Called from MainWindow whenever the 3D selection changes.
    void setSelectedEntries(const QStringList& entries);
    void setCalibrationPickAxis(const QString& axisName);

signals:
    void axisOriginChanged(const QString& axisName, double x, double y, double z);
    void calibrationFacePickRequested(const QString& targetName);
    void alignToPhysicalCenterRequested(double x, double y, double z);
    void cutterHeadModelPositionChanged(double x, double y, double z);
    void cutterHeadPhysicalPositionChanged(double x, double y, double z);
    void alignToPhysicalCutterHeadRequested();
    void axisCalibrationWizardRequested();
    void workpieceInstallPositionChanged(double x, double y, double z);
    void alignWorkpieceRotationCenterRequested();
    void autoInstallWorkpieceChanged(bool enabled);

private slots:
    void onAxisOriginEditorChanged();
    void onCutterHeadModelEditorChanged();
    void onCutterHeadPhysicalEditorChanged();
    void onWorkpieceInstallPositionChanged();

private:
    void buildUi();
    void buildConfigPage();
    void buildWorkpiecePage();
    void refreshCalibrationSection();
    void rebuildAssignmentSection();
    void rebuildWpcSection();

    LcncDocument* m_doc{nullptr};

    QWidget*    m_configPage{nullptr};

    // Static widgets
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
    QPushButton* m_btnOpenCalibrationWizard{nullptr};

    QGroupBox* m_assignGroup{nullptr};
    QLabel*    m_lblAssignSelection{nullptr};
    QGridLayout* m_assignGrid{nullptr};
    QGroupBox* m_installGroup{nullptr};
    QCheckBox* m_chkAutoInstallWorkpiece{nullptr};
    QDoubleSpinBox* m_wpcInstallX{nullptr};
    QDoubleSpinBox* m_wpcInstallY{nullptr};
    QDoubleSpinBox* m_wpcInstallZ{nullptr};
    QPushButton* m_btnAlignRotationCenter{nullptr};
    QStringList m_selectedEntries;
    QString m_pendingCalibrationAxis;
};
