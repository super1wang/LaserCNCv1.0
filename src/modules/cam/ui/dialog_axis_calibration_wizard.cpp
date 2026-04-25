#include "modules/cam/ui/dialog_axis_calibration_wizard.h"

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
    setWindowTitle(tr("机台坐标系标定向导"));
    // 非模态：用户在向导打开时仍能与 OCC 视图交互完成拾取。
    setWindowFlags(windowFlags() | Qt::Tool);
    setAttribute(Qt::WA_DeleteOnClose, false);
    buildUi();

    // 需求 1：若当前机台已有标定记录，回填物理 XYZ + AC 角度，方便用户校核或微调
    if (m_camModule) {
        gp_Pnt savedCenter;
        if (m_camModule->physicalAcCenter(savedCenter)) {
            if (m_physX) m_physX->setValue(savedCenter.X());
            if (m_physY) m_physY->setValue(savedCenter.Y());
            if (m_physZ) m_physZ->setValue(savedCenter.Z());
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "Wizard prefill: physical center=({:.3f},{:.3f},{:.3f})",
                       savedCenter.X(), savedCenter.Y(), savedCenter.Z());
        }
        double aOff = 0.0, cOff = 0.0;
        if (m_camModule->acAngleOffset(aOff, cOff)) {
            if (m_physAngleA) m_physAngleA->setValue(aOff);
            if (m_physAngleC) m_physAngleC->setValue(cOff);
            LCNC_DEBUG(lcnc::LogCode::Generic,
                       "Wizard prefill: AC angles A={:.3f}° C={:.3f}°", aOff, cOff);
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
        tr("依次拾取 A 轴、C 轴参考面，再拾取切割头下端面，最后填入物理 AC 中心后提交。\n"
           "提交时将一次性写入轴心、切割头模型点，并把模型 AC 中心搬到物理坐标处。"),
        this);
    m_lblHint->setWordWrap(true);
    m_lblHint->setStyleSheet("color: #555; font-size: 11px;");
    root->addWidget(m_lblHint);

    auto* groupPick = new QGroupBox(tr("第 1-3 步：拾取参考点"), this);
    auto* pickGrid = new QGridLayout(groupPick);
    pickGrid->setHorizontalSpacing(8);
    pickGrid->setVerticalSpacing(6);
    pickGrid->setColumnStretch(1, 1);

    m_btnPickA = new QPushButton(tr("拾取 A 轴参考面..."), groupPick);
    m_btnPickC = new QPushButton(tr("拾取 C 轴参考面..."), groupPick);
    m_btnPickHead = new QPushButton(tr("拾取切割头下端面..."), groupPick);
    m_lblAStatus = new QLabel(tr("（未拾取）"), groupPick);
    m_lblCStatus = new QLabel(tr("（未拾取）"), groupPick);
    m_lblHeadStatus = new QLabel(tr("（未拾取）"), groupPick);
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

    // ── 机台标定位（snap 显示姿态 + 实时显示当前 AC 中心 / 切割嘴） ──
    auto* groupPose = new QGroupBox(tr("机台标定位"), this);
    auto* poseLayout = new QFormLayout(groupPose);
    m_btnEnterStandardPose = new QPushButton(tr("进入机台标定位（A=0, C=0, XY 对齐）"), groupPose);
    m_btnEnterStandardPose->setEnabled(false);
    m_btnEnterStandardPose->setToolTip(
        tr("将模型显示姿态归位：A 轴角=0、C 轴角=0、XY 把切割头与 AC 中心对齐。"
           "需要先完成上方三段拾取。"));
    m_lblCurrentAcCenter = new QLabel(tr("（待进入标定位）"), groupPose);
    m_lblCurrentCutterHead = new QLabel(tr("（待进入标定位）"), groupPose);
    for (QLabel* lbl : {m_lblCurrentAcCenter, m_lblCurrentCutterHead}) {
        lbl->setStyleSheet("color:#888;");
        lbl->setMinimumWidth(280);
    }
    poseLayout->addRow(m_btnEnterStandardPose);
    poseLayout->addRow(tr("当前 AC 中心:"), m_lblCurrentAcCenter);
    poseLayout->addRow(tr("当前切割嘴 (世界):"), m_lblCurrentCutterHead);
    root->addWidget(groupPose);

    auto* groupPhys = new QGroupBox(tr("第 4 步：物理 AC 中心 + 物理 AC 角度"), this);
    auto* physForm = new QFormLayout(groupPhys);
    m_physX = makeMillimeterSpin(groupPhys);
    m_physY = makeMillimeterSpin(groupPhys);
    m_physZ = makeMillimeterSpin(groupPhys);
    m_physAngleA = new QDoubleSpinBox(groupPhys);
    m_physAngleA->setRange(-360.0, 360.0);
    m_physAngleA->setDecimals(3);
    m_physAngleA->setSingleStep(1.0);
    m_physAngleA->setSuffix(QStringLiteral(" °"));
    m_physAngleC = new QDoubleSpinBox(groupPhys);
    m_physAngleC->setRange(-360.0, 360.0);
    m_physAngleC->setDecimals(3);
    m_physAngleC->setSingleStep(1.0);
    m_physAngleC->setSuffix(QStringLiteral(" °"));
    physForm->addRow(tr("X:"), m_physX);
    physForm->addRow(tr("Y:"), m_physY);
    physForm->addRow(tr("Z:"), m_physZ);
    physForm->addRow(tr("物理 A 角度:"), m_physAngleA);
    physForm->addRow(tr("物理 C 角度:"), m_physAngleC);
    auto* lblAngleHint = new QLabel(
        tr("默认 0/0 表示该姿态对应物理机台 A=0, C=0；如标定时机台 AC 不为 0 请如实填写。"),
        groupPhys);
    lblAngleHint->setWordWrap(true);
    lblAngleHint->setStyleSheet("color:#888;font-size:11px;");
    physForm->addRow(lblAngleHint);
    root->addWidget(groupPhys);

    auto* btnRow = new QHBoxLayout();
    m_btnReset = new QPushButton(tr("重置"), this);
    m_btnSubmit = new QPushButton(tr("提交标定"), this);
    m_btnCancel = new QPushButton(tr("关闭"), this);
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
    case Stage::AAxis:      return tr("A 轴参考面");
    case Stage::CAxis:      return tr("C 轴参考面");
    case Stage::CutterHead: return tr("切割头下端面");
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
            lbl->setText(tr("（未拾取）"));
            lbl->setStyleSheet("color: #888;");
        }
    };
    setStatus(m_lblAStatus, m_aFilled, m_aCenter);
    setStatus(m_lblCStatus, m_cFilled, m_cCenter);
    setStatus(m_lblHeadStatus, m_headFilled, m_headCenter);

    if (m_awaitingPick && m_lblHint) {
        m_lblHint->setText(tr("正在拾取：%1。请在 3D 视图左键点击平面，右键 / ESC 取消。")
                               .arg(stageDisplayName(m_awaitingStage)));
        m_lblHint->setStyleSheet("color: #c98512; font-size: 11px; font-weight: bold;");
    } else {
        m_lblHint->setText(tr("依次拾取 A 轴、C 轴参考面，再拾取切割头下端面，最后填入物理 AC 中心后提交。"));
        m_lblHint->setStyleSheet("color: #555; font-size: 11px;");
    }

    if (m_btnSubmit)
        m_btnSubmit->setEnabled(m_aFilled && m_cFilled && m_headFilled && !m_awaitingPick);
    if (m_btnEnterStandardPose)
        m_btnEnterStandardPose->setEnabled(m_aFilled && m_cFilled && m_headFilled && !m_awaitingPick);

    // 顶部已/未标定徽章
    if (m_lblCalibStatus && m_camModule) {
        if (m_camModule->isMachineCalibrated()) {
            gp_Pnt c;
            m_camModule->physicalAcCenter(c);
            double aOff = 0.0, cOff = 0.0;
            m_camModule->acAngleOffset(aOff, cOff);
            m_lblCalibStatus->setText(tr(
                "● 已标定  物理 AC 中心: %1 ; 物理角度: A=%2°  C=%3°")
                .arg(formatPoint(c))
                .arg(aOff, 0, 'f', 3)
                .arg(cOff, 0, 'f', 3));
            m_lblCalibStatus->setStyleSheet(
                "color:#fff;background:#1f7a1f;padding:4px 6px;border-radius:3px;font-weight:bold;");
        } else {
            m_lblCalibStatus->setText(tr("○ 未标定  请依次完成拾取与提交以建立物理坐标系映射"));
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

    // 刷新当前 AC 中心 / 切割嘴位置（需求 3）
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
              "Standard pose displayed: AC=({:.3f},{:.3f},{:.3f}) head=({:.3f},{:.3f},{:.3f})",
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
        m_lblCurrentAcCenter->setText(tr("（待进入标定位）"));
        m_lblCurrentAcCenter->setStyleSheet("color:#888;");
    }
    if (m_lblCurrentCutterHead) {
        m_lblCurrentCutterHead->setText(tr("（待进入标定位）"));
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
    inputs.hasPhysicalCenter     = true;
    inputs.physicalAAngle        = m_physAngleA ? m_physAngleA->value() : 0.0;
    inputs.physicalCAngle        = m_physAngleC ? m_physAngleC->value() : 0.0;

    LCNC_INFO(lcnc::LogCode::Generic,
              "DialogAxisCalibrationWizard submit: physical=({:.3f},{:.3f},{:.3f}) angles=(A={:.3f}°,C={:.3f}°)",
              inputs.physicalAcCenter.X(),
              inputs.physicalAcCenter.Y(),
              inputs.physicalAcCenter.Z(),
              inputs.physicalAAngle,
              inputs.physicalCAngle);

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
