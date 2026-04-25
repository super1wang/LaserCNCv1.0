#pragma once

#include <QDialog>
#include <QString>

#include <gp_Pnt.hxx>

class CamModule;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QGroupBox;

namespace lcnc::cam::ui {

/**
 * @brief 三段式机台坐标系标定向导（VERTICAL_AC_TABLE）。
 *
 * 流程：
 *   ① 拾取 A 轴参考面 → 记录中心
 *   ② 拾取 C 轴参考面 → 记录中心
 *   ③ 拾取切割头下端面 → 记录中心
 *   ④ 输入物理 AC 中心
 *   ⑤ 点击「提交」一次性写入轴心 + 切割头模型点 + 平移整机
 *
 * 该对话框为非模态：拾取阶段由外部（MainWindow）驱动 OCC 视图取点，
 * 通过 @ref applyPickResult 回填本对话框对应阶段的结果。
 */
class DialogAxisCalibrationWizard : public QDialog
{
    Q_OBJECT
public:
    /// 标定阶段标识，用于与外部 facePick 流程对接。
    enum class Stage {
        AAxis,        ///< 步骤 1：A 轴参考面
        CAxis,        ///< 步骤 2：C 轴参考面
        CutterHead,   ///< 步骤 3：切割头下端面
    };

    explicit DialogAxisCalibrationWizard(CamModule* camModule, QWidget* parent = nullptr);
    ~DialogAxisCalibrationWizard() override;

    /// 当前等待拾取的阶段；如未在拾取中返回 nullopt 等价值（Stage::AAxis 默认）。
    bool isAwaitingPick() const { return m_awaitingPick; }
    Stage awaitingStage() const { return m_awaitingStage; }

    /// 由外部（MainWindow）在 facePick 拿到坐标后回调，用于回填当前阶段中心。
    void applyPickResult(Stage stage, const gp_Pnt& center);
    /// 拾取被取消时调用，用于复位 UI 状态。
    void cancelPickInProgress();

signals:
    /// 请求外部启动 OCC 平面拾取，用户左键点击后返回 face center。
    void pickRequested(lcnc::cam::ui::DialogAxisCalibrationWizard::Stage stage);

private slots:
    void onPickAClicked();
    void onPickCClicked();
    void onPickHeadClicked();
    void onEnterStandardPoseClicked();
    void onSubmitClicked();
    void onResetClicked();

private:
    void buildUi();
    void refreshSummary();
    void requestPick(Stage stage);
    QString stageDisplayName(Stage stage) const;

    CamModule* m_camModule{nullptr};

    // 三段中心（模型坐标）
    bool   m_aFilled{false};
    bool   m_cFilled{false};
    bool   m_headFilled{false};
    gp_Pnt m_aCenter{};
    gp_Pnt m_cCenter{};
    gp_Pnt m_headCenter{};

    // 拾取状态
    bool  m_awaitingPick{false};
    Stage m_awaitingStage{Stage::AAxis};

    // UI 控件
    QLabel*      m_lblAStatus{nullptr};
    QLabel*      m_lblCStatus{nullptr};
    QLabel*      m_lblHeadStatus{nullptr};
    QPushButton* m_btnPickA{nullptr};
    QPushButton* m_btnPickC{nullptr};
    QPushButton* m_btnPickHead{nullptr};
    QPushButton* m_btnEnterStandardPose{nullptr};
    QLabel*      m_lblCurrentAcCenter{nullptr};
    QLabel*      m_lblCurrentCutterHead{nullptr};
    QDoubleSpinBox* m_physX{nullptr};
    QDoubleSpinBox* m_physY{nullptr};
    QDoubleSpinBox* m_physZ{nullptr};
    QDoubleSpinBox* m_physAngleA{nullptr};
    QDoubleSpinBox* m_physAngleC{nullptr};
    QPushButton* m_btnSubmit{nullptr};
    QPushButton* m_btnReset{nullptr};
    QPushButton* m_btnCancel{nullptr};
    QLabel*      m_lblHint{nullptr};
    QLabel*      m_lblCalibStatus{nullptr};  ///< 顶部“已/未标定”状态指示
    bool m_standardPoseEntered{false};  ///< 是否已进入"机台标定位"
};

} // namespace lcnc::cam::ui
