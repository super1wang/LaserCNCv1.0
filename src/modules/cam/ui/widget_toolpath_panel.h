#pragma once

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
 *  3. 操作   — Global generation, contour-start picking and active-contour rebuild.
 *  4. 坐标   — Machine coordinate table for the selected contour.
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
    void setUseFaceClassification(bool enabled);
    void setShowNormals(bool on);
    void setNormalSampleStep(double mm);

    /// Show machine coordinates for contour at given index in the table.
    void showContourCoordinates(int contourIndex);
    void setActiveContour(int contourIndex);
    void refreshParameterEditors();
    ParameterScope parameterScope() const;

    double leadInLength()  const;
    double discretizationInterval() const;
    double smoothAngle()   const;
    bool   useFaceClassification() const;

signals:
    void generateRequested();
    void pickLeadInRequested();
    void recalcRequested();
    void previewToggled(bool visible);
    void leadInLengthChanged(double mm);
    void discretizationIntervalChanged(double mm);
    void smoothAngleChanged(double deg);
    void classificationModeChanged(int mode);
    void parameterScopeChanged(bool currentContour);

private:
    void buildUi();

    LaserToolpath*  m_toolpath{nullptr};

    // Parameter widgets
    QDoubleSpinBox* m_spinLeadInLength{nullptr};
    QDoubleSpinBox* m_spinDeflection{nullptr};
    QComboBox*      m_comboParameterScope{nullptr};
    QLabel*         m_labelCurrentContour{nullptr};
    QGroupBox*      m_classificationGroup{nullptr};

    // Face classification widgets
    QDoubleSpinBox* m_spinSmoothAngle{nullptr};
    QComboBox*      m_comboClassMode{nullptr};

    // Operation buttons
    QPushButton*    m_btnGenerate{nullptr};
    QPushButton*    m_btnPickLeadIn{nullptr};
    QPushButton*    m_btnRecalc{nullptr};
    QPushButton*    m_btnPreview{nullptr};


    // 法线显示参数
    QCheckBox*      m_checkShowNormals{nullptr};
    QDoubleSpinBox* m_spinNormalStep{nullptr};

    // Coordinate table
    QTableWidget*   m_coordTable{nullptr};
    int             m_activeContourIndex{-1};

signals:
    void showNormalsToggled(bool on);
    void normalSampleStepChanged(double mm);
};
