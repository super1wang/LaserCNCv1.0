#pragma once

#include <QWidget>

class LaserToolpath;
class QDoubleSpinBox;
class QComboBox;
class QListWidget;
class QTableWidget;
class QPushButton;
class QGroupBox;

/**
 * @brief Right-panel widget for laser toolpath parameters.
 *
 * Sections:
 *  1. 参数   — Lead-in length (mm) and normal angle (°) spinboxes.
 *  2. 面分类 — Smooth angle threshold and classification mode.
 *  3. 操作   — Buttons: 生成刀路 / 选择引刀位置 / 重新计算 / 刀路预览.
 *  4. 轮廓   — Checkable list of extracted contours with enable/disable.
 */
class WidgetToolpathPanel : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetToolpathPanel(QWidget* parent = nullptr);

    /// Populate the contour list from the current toolpath data.
    void setToolpath(LaserToolpath* tp);

    /// Refresh the contour list display (checked state, names).
    void updateContourList();

    /// Show machine coordinates for contour at given index in the table.
    void showContourCoordinates(int contourIndex);

    double leadInLength()  const;
    double normalAngle()   const;
    double smoothAngle()   const;
    bool   useFaceClassification() const;

signals:
    void generateRequested();
    void pickLeadInRequested();
    void recalcRequested();
    void previewToggled(bool visible);
    void leadInLengthChanged(double mm);
    void normalAngleChanged(double deg);
    void contourToggled(int index, bool enabled);
    void smoothAngleChanged(double deg);
    void classificationModeChanged(int mode);
    void contourListUpdated();

private:
    void buildUi();

    LaserToolpath*  m_toolpath{nullptr};

    // Parameter widgets
    QDoubleSpinBox* m_spinLeadInLength{nullptr};
    QDoubleSpinBox* m_spinNormalAngle{nullptr};

    // Face classification widgets
    QDoubleSpinBox* m_spinSmoothAngle{nullptr};
    QComboBox*      m_comboClassMode{nullptr};

    // Operation buttons
    QPushButton*    m_btnGenerate{nullptr};
    QPushButton*    m_btnPickLeadIn{nullptr};
    QPushButton*    m_btnRecalc{nullptr};
    QPushButton*    m_btnPreview{nullptr};

    // Contour list
    QListWidget*    m_contourList{nullptr};

    // Coordinate table
    QTableWidget*   m_coordTable{nullptr};
};
