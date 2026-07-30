#pragma once

#include "core/kinematics/machine_configuration_service.h"
#include "core/settings/app_settings.h"

#include <QColor>
#include <QDialog>
#include <QHash>
#include <QString>
#include <QVariant>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QEvent;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QTableWidget;
class QTreeWidget;

namespace lcnc {

/**
 * @brief 应用程序选项对话框。
 *
 * 对话框采用本地 draft 缓冲：控件修改不会立即写入 AppSettings，也不会触发重绘。
 * 只有点击 Apply/OK 时才统一 diff、保存并向 RenderingManager 投递最小 dirty flags。
 */
class DialogOptions : public QDialog
{
    Q_OBJECT
public:
    /// 构造左侧导航 + 右侧参数页的应用程序选项对话框。
    explicit DialogOptions(QWidget* parent = nullptr);
    ~DialogOptions() override;

    /// 拦截 spin 控件滚轮，避免滚动页面时误改参数。
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    /// 将 draft 与 AppSettings 做差异比较，保存并投递必要渲染刷新。
    bool applyChanges();

private:
    struct RenderControls {
        QComboBox* defaultDisplay{nullptr};
        QComboBox* quality{nullptr};
        QComboBox* renderMethod{nullptr};
        QComboBox* material{nullptr};
        QCheckBox* antiAliasing{nullptr};
        QComboBox* msaaSamples{nullptr};
        QCheckBox* shadows{nullptr};
        QCheckBox* reflections{nullptr};
        QCheckBox* adaptiveSampling{nullptr};
        QCheckBox* frustumCulling{nullptr};
        QCheckBox* backFaceCulling{nullptr};
        QCheckBox* geometryMerge{nullptr};
        QCheckBox* proxyGeometry{nullptr};
        QCheckBox* lowLodWhileMoving{nullptr};
        QCheckBox* disableHeavyEffectsDuringSimulation{nullptr};
        QDoubleSpinBox* ambientLight{nullptr};
        QDoubleSpinBox* deviationCoefficient{nullptr};
        QDoubleSpinBox* deviationAngle{nullptr};
        QDoubleSpinBox* edgeWidth{nullptr};
        QDoubleSpinBox* renderResolutionScale{nullptr};
        QSpinBox* raytracingDepth{nullptr};
        QSpinBox* rayTracingTileSize{nullptr};
        QSpinBox* rayTracingTileCount{nullptr};
        QSpinBox* targetFps{nullptr};
    };

    void buildUi();
    void buildRenderPage(const QString& title, bool camView, RenderControls& controls);
    void buildColorPage();
    void buildApplicationPage();
    void buildMachineConfigurationPage();
    void disableSpinWheel(QWidget* root);
    void loadFromSettings();
    void populateMachineAxisTable(const QVector<MachineAxisRuntimeConfig>& configs);
    QList<MachineAxisDef> collectMachineAxisDefinitions() const;
    void setRotationCenterUiFromAxes(const QList<MachineAxisDef>& axes);
    void applyRotationCenterToMachineAxisTable();
    bool hasRotaryAxisInTable() const;
    void setProfileToUi(const RenderProfileSettings& profile, const RenderControls& controls);
    RenderProfileSettings collectProfileFromUi(const RenderControls& controls) const;
    void wireRenderPresetBehavior(RenderControls& controls, bool camView);
    void setComboByData(QComboBox* combo, const QVariant& value);
    QPushButton* makeColorButton(QColor* target);
    void applyTreeSelectionColor(const QColor& color);

    QTreeWidget*    m_nav{nullptr};
    QStackedWidget* m_stack{nullptr};

    RenderControls m_renderControls;

    QPushButton* m_btnWorkpieceColor{nullptr};
    QPushButton* m_btnBackgroundColor{nullptr};
    QDoubleSpinBox* m_spWorkpieceTransparency{nullptr};
    QDoubleSpinBox* m_spMachineTransparency{nullptr};
    QPushButton* m_btnSelectionColor{nullptr};
    QPushButton* m_btnHoverColor{nullptr};
    QPushButton* m_btnTreeSelectionColor{nullptr};
    QComboBox* m_cbHighlightMode{nullptr};
    QDoubleSpinBox* m_spHighlightLineWidth{nullptr};
    QHash<QString, QPushButton*> m_axisColorButtons;

    QComboBox* m_cbLanguage{nullptr};
    QComboBox* m_cbTheme{nullptr};
    QComboBox* m_cbUnits{nullptr};
    QComboBox* m_cbDocumentOpenMode{nullptr};
    QSpinBox* m_spRecentLimit{nullptr};

    QComboBox* m_cbMachinePreset{nullptr};
    QLineEdit* m_editMachineModelPath{nullptr};
    QPushButton* m_btnBrowseMachineModel{nullptr};
    QCheckBox* m_chkAutoLoadMachineModel{nullptr};
    QLabel* m_lblMachineAlgorithm{nullptr};
    QLabel* m_lblRotationCenterHint{nullptr};
    QDoubleSpinBox* m_spRotationCenterX{nullptr};
    QDoubleSpinBox* m_spRotationCenterY{nullptr};
    QDoubleSpinBox* m_spRotationCenterZ{nullptr};
    QTableWidget* m_machineAxesTable{nullptr};
    MachineConfigurationService* m_machineConfig{nullptr};

    RenderProfileSettings m_renderDraft;
    RenderProfileSettings m_originalCad;
    RenderProfileSettings m_originalCam;
    ColorSettings m_colorDraft;
    ColorSettings m_originalColors;
    QString m_originalLanguage;
    QString m_originalTheme;
    QString m_originalUnitSystem;
    DocumentOpenMode m_originalDocumentOpenMode{DocumentOpenMode::MultiDocument};
    int m_originalRecentLimit{10};
    QString m_originalMachineModelPath;
    bool m_originalAutoLoadMachineModel{true};
    QString m_originalMachinePreset;
    QVector<MachineAxisRuntimeConfig> m_originalMachineConfigs;
    bool m_loadingUi{false};
};

} // namespace lcnc
