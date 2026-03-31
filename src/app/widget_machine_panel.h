#pragma once

#include <QWidget>
#include <QString>
#include <QMap>
#include <QStringList>

class LcncDocument;
class MachineKinematics;
class QLabel;
class QComboBox;
class QFormLayout;
class QGroupBox;
class QDoubleSpinBox;

/**
 * @brief Right-panel widget shown when the "准备" tab is active.
 *
 * Sections:
 *  1. 机台配置   — config-type combo + "加载机台" + "标记轴系" + machine name label.
 *  2. 轴系位置   — dynamically built spinbox row per axis (Linear: mm, Rotary: °).
 *  3. 工件挂载   — workpiece drop-down + target-axis drop-down + "挂载"/"解除" buttons.
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

signals:
    void loadMachineRequested();
    void markAxesRequested();
    void mountWorkpieceRequested();
    void unloadMachineRequested();
    void exportMachineRequested();
    /// Emitted when the user adjusts an axis-position spinbox.
    void axisPositionChanged(const QString& axisName, double valueMmOrDeg);
    /// Emitted after a shape has been (re)assigned to an axis via the mark buttons.
    void axisAssignmentChanged();

private slots:
    void onAxisSpinChanged(const QString& axisName, double value);

private:
    void buildUi();
    void rebuildAxisRows();
    void rebuildWpcSection();
    void rebuildMarkButtons();

    LcncDocument* m_doc{nullptr};

    // Static widgets
    QLabel*    m_lblMachineName{nullptr};
    QLabel*    m_lblConfigType{nullptr};
    QGroupBox* m_axisGroup{nullptr};
    QFormLayout* m_axisLayout{nullptr};
    QGroupBox* m_wpcGroup{nullptr};
    QFormLayout* m_wpcLayout{nullptr};

    // Dynamically created: axisName → spinbox
    QMap<QString, QDoubleSpinBox*> m_axisSpin;

    // ── Mark-shape section ──────────────────────────────────────────────────
    QGroupBox* m_markGroup{nullptr};
    QWidget*   m_markWidget{nullptr};
    QStringList m_selectedEntries;
};
