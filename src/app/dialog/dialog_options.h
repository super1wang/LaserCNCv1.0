#pragma once

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
class QPushButton;
class QSpinBox;
class QStackedWidget;
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
    void disableSpinWheel(QWidget* root);
    void loadFromSettings();
    void setProfileToUi(const RenderProfileSettings& profile, const RenderControls& controls);
    RenderProfileSettings collectProfileFromUi(const RenderControls& controls) const;
    void wireRenderPresetBehavior(RenderControls& controls, bool camView);
    void setComboByData(QComboBox* combo, const QVariant& value);
    QPushButton* makeColorButton(QColor* target);
    void applyTreeSelectionColor(const QColor& color);

    QTreeWidget*    m_nav{nullptr};
    QStackedWidget* m_stack{nullptr};

    RenderControls m_cadControls;
    RenderControls m_camControls;

    QPushButton* m_btnWorkpieceColor{nullptr};
    QPushButton* m_btnCadBackground{nullptr};
    QPushButton* m_btnCamBackground{nullptr};
    QPushButton* m_btnSelectionColor{nullptr};
    QPushButton* m_btnHoverColor{nullptr};
    QPushButton* m_btnTreeSelectionColor{nullptr};
    QComboBox* m_cbHighlightMode{nullptr};
    QDoubleSpinBox* m_spHighlightLineWidth{nullptr};
    QHash<QString, QPushButton*> m_axisColorButtons;

    QComboBox* m_cbLanguage{nullptr};
    QComboBox* m_cbTheme{nullptr};
    QComboBox* m_cbUnits{nullptr};
    QSpinBox* m_spRecentLimit{nullptr};

    RenderProfileSettings m_cadDraft;
    RenderProfileSettings m_camDraft;
    ColorSettings m_colorDraft;
    RenderProfileSettings m_originalCad;
    RenderProfileSettings m_originalCam;
    ColorSettings m_originalColors;
    QString m_originalLanguage;
    QString m_originalTheme;
    QString m_originalUnitSystem;
    int m_originalRecentLimit{10};
    bool m_loadingUi{false};
};

} // namespace lcnc
