#include "modules/cam/ui/machine/dialog_axis_calibration_wizard.h"

#include "modules/cam/cam_module.h"
#include "core/logging/logger.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace lcnc::cam::ui {

namespace {

QString formatPoint(const gp_Pnt& p)
{
    return QObject::tr("X=%1  Y=%2  Z=%3")
        .arg(p.X(), 0, 'f', 3)
        .arg(p.Y(), 0, 'f', 3)
        .arg(p.Z(), 0, 'f', 3);
}

QDoubleSpinBox* makeMillimeterSpin(QWidget* parent)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(-99999.0, 99999.0);
    spin->setDecimals(3);
    spin->setSingleStep(1.0);
    spin->setSuffix(QStringLiteral(" mm"));
    spin->setMinimumWidth(120);
    return spin;
}

} // namespace

DialogAxisCalibrationWizard::DialogAxisCalibrationWizard(CamModule* camModule, QWidget* parent)
    : QDialog(parent)
    , m_camModule(camModule)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogAxisCalibrationWizard ctor");
    // 中文翻译：机台坐标系标定向导
    setWindowTitle(tr("Machine coordinate system calibration wizard"));
    // 非模态：用户在向导打开时仍能与 OCC 视图交互完成拾取。
    setWindowFlags(windowFlags() | Qt::Tool);
    setAttribute(Qt::WA_DeleteOnClose, false);
    buildUi();

    // 旋转中心由“应用程序选项 / 机台构型”统一维护；向导只读取用于对齐提示。
    if (m_camModule) {
        gp_Pnt savedCenter;
        if (m_camModule->currentAcRotationCenter(savedCenter)) {
            if (m_physX) m_physX->setValue(savedCenter.X());
            if (m_physY) m_physY->setValue(savedCenter.Y());
            if (m_physZ) m_physZ->setValue(savedCenter.Z());
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "Wizard prefill: configured rotation center=({:.3f},{:.3f},{:.3f})",
                       savedCenter.X(), savedCenter.Y(), savedCenter.Z());
        }
    }

    refreshSummary();
}

DialogAxisCalibrationWizard::~DialogAxisCalibrationWizard() = default;

void DialogAxisCalibrationWizard::buildUi()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(10);

    // ── 顶部状态：是否已标定（需求 1，提示用户当前机台标定历史） ──
    m_lblCalibStatus = new QLabel(this);
    m_lblCalibStatus->setWordWrap(true);
    root->addWidget(m_lblCalibStatus);

    m_lblHint = new QLabel(
        // 中文翻译：依次拾取 A 轴、C 轴参考面，再拾取切割头下端面。\n
        tr("Select the A-axis and C-axis reference surfaces in sequence, and then select the lower end surface of the cutting head."
        // 中文翻译：提交时先牵引整机，使 A 面 Y/Z 与 C 面 X 合成的模型 AC 中心对齐绝对 AC 中心；再按运动子树分别校正：Y 带动 Y/X/Z，X 带动 X/Z，Z 只移动 Z 轴滑台，使所选刀嘴面的 XYZ 对齐当前机台 XYZ 所代表的绝对模拟锥头 TCP。实时轴坐标不会改变。
           "\nOn submission, the complete machine is first dragged so that the model AC center composed from A-face Y/Z and C-face X reaches the absolute AC center. Then corrections are propagated through the kinematic subtrees: Y moves Y/X/Z, X moves X/Z, and Z moves only the Z slide, so the picked cutter-face XYZ reaches the absolute simulated TCP represented by the current live machine XYZ. Live axis coordinates are never changed."),
        this);
    m_lblHint->setWordWrap(true);
    m_lblHint->setStyleSheet("color: #555; font-size: 11px;");
    root->addWidget(m_lblHint);

    // 中文翻译：第 1-3 步：拾取参考点
    auto* groupPick = new QGroupBox(tr("Steps 1-3: Pick a reference point"), this);
    auto* pickGrid = new QGridLayout(groupPick);
    pickGrid->setHorizontalSpacing(8);
    pickGrid->setVerticalSpacing(6);
    pickGrid->setColumnStretch(1, 1);

    // 中文翻译：拾取 A 轴参考面...
    m_btnPickA = new QPushButton(tr("Pick the A-axis reference plane..."), groupPick);
    // 中文翻译：拾取 C 轴参考面...
    m_btnPickC = new QPushButton(tr("Pick the C-axis reference plane..."), groupPick);
    // 中文翻译：拾取切割头下端面...
    m_btnPickHead = new QPushButton(tr("Pick up the lower end face of the cutting head..."), groupPick);
    // 中文翻译：（未拾取）
    m_lblAStatus = new QLabel(tr("(not picked up)"), groupPick);
    // 中文翻译：（未拾取）
    m_lblCStatus = new QLabel(tr("(not picked up)"), groupPick);
    // 中文翻译：（未拾取）
    m_lblHeadStatus = new QLabel(tr("(not picked up)"), groupPick);
    for (QLabel* lbl : {m_lblAStatus, m_lblCStatus, m_lblHeadStatus}) {
        lbl->setStyleSheet("color: #888;");
        lbl->setMinimumWidth(280);
    }

    pickGrid->addWidget(m_btnPickA,    0, 0);
    pickGrid->addWidget(m_lblAStatus,  0, 1);
    pickGrid->addWidget(m_btnPickC,    1, 0);
    pickGrid->addWidget(m_lblCStatus,  1, 1);
    pickGrid->addWidget(m_btnPickHead, 2, 0);
    pickGrid->addWidget(m_lblHeadStatus, 2, 1);
    root->addWidget(groupPick);

    // ── 绝对标定目标（只读，不改变实时轴位姿） ──
    // 中文翻译：绝对标定目标
    auto* groupPose = new QGroupBox(tr("Absolute calibration target"), this);
    auto* poseLayout = new QFormLayout(groupPose);
    // 中文翻译：确认绝对 AC 中心与模拟锥头目标
    m_btnEnterStandardPose = new QPushButton(tr("Confirm absolute AC-center and simulated-TCP targets"), groupPose);
    m_btnEnterStandardPose->setEnabled(false);
    m_btnEnterStandardPose->setToolTip(
        // 中文翻译：只读取并显示绝对世界坐标目标，不修改 A/C/X/Y/Z 实时轴坐标。
        tr("Read and display the absolute world-coordinate targets only; do not change live A/C/X/Y/Z axis coordinates."
           // 中文翻译：需要先完成上方三段拾取。
           "\nYou need to complete the three pickups above first."));
    // 中文翻译：（待确认绝对目标）
    m_lblCurrentAcCenter = new QLabel(tr("(Absolute targets not confirmed)"), groupPose);
    // 中文翻译：（待确认绝对目标）
    m_lblCurrentCutterHead = new QLabel(tr("(Absolute targets not confirmed)"), groupPose);
    for (QLabel* lbl : {m_lblCurrentAcCenter, m_lblCurrentCutterHead}) {
        lbl->setStyleSheet("color:#888;");
        lbl->setMinimumWidth(280);
    }
    poseLayout->addRow(m_btnEnterStandardPose);
    // 中文翻译：当前 AC 中心:
    poseLayout->addRow(tr("Current AC Center:"), m_lblCurrentAcCenter);
    // 中文翻译：绝对模拟锥头 TCP:
    poseLayout->addRow(tr("Absolute simulated cutter TCP:"), m_lblCurrentCutterHead);
    root->addWidget(groupPose);

    // 中文翻译：第 4 步：确认构型旋转中心（只读）
    auto* groupPhys = new QGroupBox(tr("Step 4: Confirm configuration center of rotation (read only)"), this);
    auto* physForm = new QFormLayout(groupPhys);
    m_physX = makeMillimeterSpin(groupPhys);
    m_physY = makeMillimeterSpin(groupPhys);
    m_physZ = makeMillimeterSpin(groupPhys);
    m_physX->setEnabled(false);
    m_physY->setEnabled(false);
    m_physZ->setEnabled(false);
    // 中文翻译：旋转中心 X:
    physForm->addRow(tr("Center of rotation X:"), m_physX);
    // 中文翻译：旋转中心 Y:
    physForm->addRow(tr("Rotation center Y:"), m_physY);
    // 中文翻译：旋转中心 Z:
    physForm->addRow(tr("Center of rotation Z:"), m_physZ);
    auto* lblAngleHint = new QLabel(
        // 中文翻译：如需修改旋转中心，请在“应用程序选项 / 机台构型”页填写；本向导不会修改物理中心。
        tr("If you need to modify the rotation center, please fill it in the \"Application Options/Machine Configuration\" page; this wizard will not modify the physical center."),
        groupPhys);
    lblAngleHint->setWordWrap(true);
    lblAngleHint->setStyleSheet("color:#888;font-size:11px;");
    physForm->addRow(lblAngleHint);
    root->addWidget(groupPhys);

    auto* btnRow = new QHBoxLayout();
    // 中文翻译：重置
    m_btnReset = new QPushButton(tr("reset"), this);
    // 中文翻译：提交标定
    m_btnSubmit = new QPushButton(tr("Submit calibration"), this);
    // 中文翻译：关闭
    m_btnCancel = new QPushButton(tr("Close"), this);
    m_btnSubmit->setDefault(true);
    btnRow->addWidget(m_btnReset);
    btnRow->addStretch(1);
    btnRow->addWidget(m_btnSubmit);
    btnRow->addWidget(m_btnCancel);
    root->addLayout(btnRow);

    connect(m_btnPickA, &QPushButton::clicked, this, &DialogAxisCalibrationWizard::onPickAClicked);
    connect(m_btnPickC, &QPushButton::clicked, this, &DialogAxisCalibrationWizard::onPickCClicked);
    connect(m_btnPickHead, &QPushButton::clicked, this, &DialogAxisCalibrationWizard::onPickHeadClicked);
    connect(m_btnEnterStandardPose, &QPushButton::clicked,
            this, &DialogAxisCalibrationWizard::onEnterStandardPoseClicked);
    connect(m_btnSubmit, &QPushButton::clicked, this, &DialogAxisCalibrationWizard::onSubmitClicked);
    connect(m_btnReset, &QPushButton::clicked, this, &DialogAxisCalibrationWizard::onResetClicked);
    connect(m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);
}

QString DialogAxisCalibrationWizard::stageDisplayName(Stage stage) const
{
    switch (stage) {
    // 中文翻译：A 轴参考面
    case Stage::AAxis:      return tr("A-axis reference plane");
    // 中文翻译：C 轴参考面
    case Stage::CAxis:      return tr("C-axis reference plane");
    // 中文翻译：切割头下端面
    case Stage::CutterHead: return tr("Lower end of cutting head");
    }
    return {};
}

void DialogAxisCalibrationWizard::onPickAClicked()    { requestPick(Stage::AAxis); }
void DialogAxisCalibrationWizard::onPickCClicked()    { requestPick(Stage::CAxis); }
void DialogAxisCalibrationWizard::onPickHeadClicked() { requestPick(Stage::CutterHead); }

void DialogAxisCalibrationWizard::requestPick(Stage stage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "DialogAxisCalibrationWizard::requestPick stage={}",
               static_cast<int>(stage));
    m_awaitingPick = true;
    m_awaitingStage = stage;
    refreshSummary();
    emit pickRequested(stage);
}

void DialogAxisCalibrationWizard::applyPickResult(Stage stage, const gp_Pnt& center)
{
    LCNC_INFO(lcnc::LogCode::Generic,
              "DialogAxisCalibrationWizard::applyPickResult stage={} ({:.3f},{:.3f},{:.3f})",
              static_cast<int>(stage), center.X(), center.Y(), center.Z());
    switch (stage) {
    case Stage::AAxis:      m_aCenter = center;    m_aFilled = true;    break;
    case Stage::CAxis:      m_cCenter = center;    m_cFilled = true;    break;
    case Stage::CutterHead: m_headCenter = center; m_headFilled = true; break;
    }
    m_awaitingPick = false;
    refreshSummary();
}

void DialogAxisCalibrationWizard::cancelPickInProgress()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogAxisCalibrationWizard::cancelPickInProgress");
    m_awaitingPick = false;
    refreshSummary();
}

void DialogAxisCalibrationWizard::refreshSummary()
{
    auto setStatus = [](QLabel* lbl, bool filled, const gp_Pnt& p) {
        if (!lbl) return;
        if (filled) {
            lbl->setText(formatPoint(p));
            lbl->setStyleSheet("color: #1f7a1f;");
        } else {
            // 中文翻译：（未拾取）
            lbl->setText(tr("(not picked up)"));
            lbl->setStyleSheet("color: #888;");
        }
    };
    setStatus(m_lblAStatus, m_aFilled, m_aCenter);
    setStatus(m_lblCStatus, m_cFilled, m_cCenter);
    setStatus(m_lblHeadStatus, m_headFilled, m_headCenter);

    if (m_awaitingPick && m_lblHint) {
        // 中文翻译：正在拾取：%1。请在 3D 视图左键点击平面，右键 / ESC 取消。
        m_lblHint->setText(tr("Picking up: %1. Please left-click the plane in the 3D view, right-click/ESC to cancel.")
                               .arg(stageDisplayName(m_awaitingStage)));
        m_lblHint->setStyleSheet("color: #c98512; font-size: 11px; font-weight: bold;");
    } else {
        // 中文翻译：依次拾取 A 轴、C 轴参考面，再拾取切割头下端面，然后提交模型对齐。
        m_lblHint->setText(tr("Select the A-axis and C-axis reference surfaces in sequence, then select the lower end surface of the cutting head, and then submit the model for alignment."));
        m_lblHint->setStyleSheet("color: #555; font-size: 11px;");
    }

    if (m_btnSubmit)
        m_btnSubmit->setEnabled(m_aFilled && m_cFilled && m_headFilled && !m_awaitingPick);
    if (m_btnEnterStandardPose)
        m_btnEnterStandardPose->setEnabled(m_aFilled && m_cFilled && m_headFilled && !m_awaitingPick);

    // 顶部已/未标定徽章
    if (m_lblCalibStatus && m_camModule) {
        gp_Pnt c;
        if (m_camModule->currentAcRotationCenter(c)) {
            m_lblCalibStatus->setText(tr(
                // 中文翻译：● 当前构型旋转中心: %1
                "● Current configuration rotation center: %1").arg(formatPoint(c)));
            m_lblCalibStatus->setStyleSheet(
                "color:#fff;background:#1f7a1f;padding:4px 6px;border-radius:3px;font-weight:bold;");
        } else {
            // 中文翻译：○ 尚未配置旋转中心  请先到应用程序选项 / 机台构型页填写
            m_lblCalibStatus->setText(tr("○ The rotation center has not been configured yet. Please go to the application options/machine configuration page to fill it in first."));
            m_lblCalibStatus->setStyleSheet(
                "color:#fff;background:#a55;padding:4px 6px;border-radius:3px;font-weight:bold;");
        }
    }
}

void DialogAxisCalibrationWizard::onEnterStandardPoseClicked()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogAxisCalibrationWizard::onEnterStandardPoseClicked");
    if (!m_camModule || !m_aFilled || !m_cFilled || !m_headFilled)
        return;

    CamModule::AxisCalibrationInputs inputs;
    inputs.aFaceCenter          = m_aCenter;
    inputs.cFaceCenter          = m_cCenter;
    inputs.cutterHeadFaceCenter = m_headCenter;
    inputs.hasPhysicalCenter    = false;

    QString errMsg;
    if (!m_camModule->enterStandardCalibrationPose(inputs, &errMsg)) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Standard pose entry failed: {}", errMsg.toStdString());
        return;
    }
    m_standardPoseEntered = true;

    // 显示不可变的绝对 AC 中心与当前实时 XYZ 所代表的绝对 TCP。
    gp_Pnt acCenter;
    if (m_camModule->currentAcRotationCenter(acCenter) && m_lblCurrentAcCenter) {
        m_lblCurrentAcCenter->setText(formatPoint(acCenter));
        m_lblCurrentAcCenter->setStyleSheet("color:#1f7a1f;");
    }
    const gp_Pnt headWorld = m_camModule->cutterHeadWorldPosition();
    if (m_lblCurrentCutterHead) {
        m_lblCurrentCutterHead->setText(formatPoint(headWorld));
        m_lblCurrentCutterHead->setStyleSheet("color:#1f7a1f;");
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "Absolute calibration targets displayed: AC=({:.3f},{:.3f},{:.3f}) TCP=({:.3f},{:.3f},{:.3f})",
              acCenter.X(), acCenter.Y(), acCenter.Z(),
              headWorld.X(), headWorld.Y(), headWorld.Z());
}

void DialogAxisCalibrationWizard::onResetClicked()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogAxisCalibrationWizard::onResetClicked");
    m_aFilled = m_cFilled = m_headFilled = false;
    m_aCenter = m_cCenter = m_headCenter = gp_Pnt(0.0, 0.0, 0.0);
    m_awaitingPick = false;
    m_standardPoseEntered = false;
    if (m_lblCurrentAcCenter) {
        // 中文翻译：（待确认绝对目标）
        m_lblCurrentAcCenter->setText(tr("(Absolute targets not confirmed)"));
        m_lblCurrentAcCenter->setStyleSheet("color:#888;");
    }
    if (m_lblCurrentCutterHead) {
        // 中文翻译：（待确认绝对目标）
        m_lblCurrentCutterHead->setText(tr("(Absolute targets not confirmed)"));
        m_lblCurrentCutterHead->setStyleSheet("color:#888;");
    }
    refreshSummary();
}

void DialogAxisCalibrationWizard::onSubmitClicked()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "DialogAxisCalibrationWizard::onSubmitClicked");
    if (!m_camModule) {
        LCNC_ERR(lcnc::LogCode::Generic, "DialogAxisCalibrationWizard: m_camModule is null");
        return;
    }
    if (!m_aFilled || !m_cFilled || !m_headFilled) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "DialogAxisCalibrationWizard::onSubmitClicked rejected (incomplete picks)");
        return;
    }

    CamModule::AxisCalibrationInputs inputs;
    inputs.aFaceCenter           = m_aCenter;
    inputs.cFaceCenter           = m_cCenter;
    inputs.cutterHeadFaceCenter  = m_headCenter;
    inputs.physicalAcCenter      = gp_Pnt(m_physX->value(), m_physY->value(), m_physZ->value());
    inputs.hasPhysicalCenter     = false;
    inputs.physicalAAngle        = 0.0;
    inputs.physicalCAngle        = 0.0;

    LCNC_INFO(lcnc::LogCode::Generic,
              "DialogAxisCalibrationWizard submit: configuredCenter=({:.3f},{:.3f},{:.3f})",
              inputs.physicalAcCenter.X(),
              inputs.physicalAcCenter.Y(),
              inputs.physicalAcCenter.Z());

    QString errorMessage;
    if (m_camModule->applyAxisCalibration(inputs, &errorMessage)) {
        // 需求 4：提交后刷新显示，验证当前坐标已与物理坐标系同步
        gp_Pnt acAfter;
        if (m_camModule->currentAcRotationCenter(acAfter) && m_lblCurrentAcCenter) {
            m_lblCurrentAcCenter->setText(formatPoint(acAfter));
            m_lblCurrentAcCenter->setStyleSheet("color:#1f7a1f;font-weight:bold;");
        }
        const gp_Pnt headAfter = m_camModule->cutterHeadWorldPosition();
        if (m_lblCurrentCutterHead) {
            m_lblCurrentCutterHead->setText(formatPoint(headAfter));
            m_lblCurrentCutterHead->setStyleSheet("color:#1f7a1f;font-weight:bold;");
        }
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Calibration verified post-submit: AC=({:.3f},{:.3f},{:.3f}) head=({:.3f},{:.3f},{:.3f})",
                  acAfter.X(), acAfter.Y(), acAfter.Z(),
                  headAfter.X(), headAfter.Y(), headAfter.Z());
        accept();
    } else {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "Axis calibration rejected: {}", errorMessage.toStdString());
        // operationFailed 信号已由 CamModule 弹窗，无需在此 QMessageBox。
    }
}

} // namespace lcnc::cam::ui
