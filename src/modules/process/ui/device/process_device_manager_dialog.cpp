#include "modules/process/ui/device/process_device_manager_dialog.h"

#include "modules/process/device/i_laser_device.h"
#include "modules/process/device/i_process_io.h"
#include "modules/process/device/process_device_manager.h"
#include "modules/process/settings/process_settings.h"
#include "core/kernel/kernel.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSet>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QUiLoader>
#include <QVBoxLayout>
#include <QtConcurrent>

#include <algorithm>
#include <functional>

namespace lcnc::process {
namespace {

class DeviceUiLoader : public QUiLoader
{
public:
    using QUiLoader::QUiLoader;

protected:
    QWidget* createWidget(const QString& className, QWidget* parent, const QString& name) override
    {
        if (className == QStringLiteral("CSwitchWidget")) {
            auto* widget = new QCheckBox(parent);
            widget->setObjectName(name);
            return widget;
        }
        return QUiLoader::createWidget(className, parent, name);
    }
};

QLabel* valueLabel(const QString& text, QWidget* parent)
{
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return label;
}

QString editorValue(QWidget* editor)
{
    if (auto* check = qobject_cast<QCheckBox*>(editor))
        return check->isChecked() ? QStringLiteral("true") : QStringLiteral("false");
    if (auto* combo = qobject_cast<QComboBox*>(editor))
        return combo->currentText();
    if (auto* spin = qobject_cast<QSpinBox*>(editor))
        return QString::number(spin->value());
    if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor))
        return QString::number(spin->value(), 'g', 15);
    if (auto* edit = qobject_cast<QLineEdit*>(editor))
        return edit->text().trimmed();
    if (auto* text = qobject_cast<QTextEdit*>(editor))
        return text->toPlainText().trimmed();
    if (auto* text = qobject_cast<QPlainTextEdit*>(editor))
        return text->toPlainText().trimmed();
    return QString();
}

void setEditorValue(QWidget* editor, const QString& value)
{
    if (auto* check = qobject_cast<QCheckBox*>(editor))
        check->setChecked(value == QStringLiteral("true") || value == QStringLiteral("1"));
    else if (auto* combo = qobject_cast<QComboBox*>(editor)) {
        const int index = combo->findText(value);
        if (index >= 0)
            combo->setCurrentIndex(index);
    } else if (auto* spin = qobject_cast<QSpinBox*>(editor))
        spin->setValue(value.toInt());
    else if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor))
        spin->setValue(value.toDouble());
    else if (auto* edit = qobject_cast<QLineEdit*>(editor))
        edit->setText(value);
    else if (auto* text = qobject_cast<QTextEdit*>(editor))
        text->setPlainText(value);
    else if (auto* text = qobject_cast<QPlainTextEdit*>(editor))
        text->setPlainText(value);
}

bool isEditableWidget(QWidget* widget)
{
    return (qobject_cast<QLineEdit*>(widget)
            || qobject_cast<QComboBox*>(widget)
            || qobject_cast<QCheckBox*>(widget)
            || qobject_cast<QSpinBox*>(widget)
            || qobject_cast<QDoubleSpinBox*>(widget)
            || qobject_cast<QTextEdit*>(widget)
            || qobject_cast<QPlainTextEdit*>(widget))
        && !widget->objectName().trimmed().isEmpty();
}

QString ioNameFromLegacyField(QString fieldName, const QString& prefix)
{
    fieldName.remove(prefix);
    if (fieldName.startsWith(QLatin1Char('a')) && fieldName.size() > 1)
        fieldName.remove(0, 1);
    return fieldName.replace(QLatin1Char('_'), QStringLiteral(" ")).trimmed();
}

QString directionText(bool output)
{
    return output ? QStringLiteral("输出") : QStringLiteral("输入");
}

QComboBox* makeDirectionCombo(QWidget* parent, bool output)
{
    auto* combo = new QComboBox(parent);
    combo->addItem(QStringLiteral("输入"), false);
    combo->addItem(QStringLiteral("输出"), true);
    combo->setCurrentIndex(output ? 1 : 0);
    return combo;
}

QTableWidgetItem* axisTableItem(const QString& text, bool editable = true)
{
    auto* item = new QTableWidgetItem(text);
    if (!editable)
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    return item;
}

void insertAxisConfigRow(QTableWidget* table, const lcnc::MachineAxisRuntimeConfig& config)
{
    const int row = table->rowCount();
    table->insertRow(row);
    table->setItem(row, 0, axisTableItem(config.axis.name, false));
    table->setItem(row, 1, axisTableItem(QString::number(config.controllerIndex)));
    table->setItem(row, 2, axisTableItem(QString::number(config.homeIndex)));
    table->setItem(row, 3, axisTableItem(QString::number(config.lowSpeed, 'g', 15)));
    table->setItem(row, 4, axisTableItem(QString::number(config.mediumSpeed, 'g', 15)));
    table->setItem(row, 5, axisTableItem(QString::number(config.highSpeed, 'g', 15)));
    table->setItem(row, 6, axisTableItem(QString::number(config.acceleration, 'g', 15)));
    table->setItem(row, 7, axisTableItem(QString::number(config.jerk, 'g', 15)));
    table->setItem(row, 8, axisTableItem(QString::number(config.axis.minVal, 'g', 15)));
    table->setItem(row, 9, axisTableItem(QString::number(config.axis.maxVal, 'g', 15)));
}

void insertIoTableRow(QTableWidget* table, const lcnc::ProcessIoTableEntry& entry)
{
    const int row = table->rowCount();
    table->insertRow(row);
    table->setItem(row, 0, new QTableWidgetItem(entry.name));
    table->setItem(row, 1, new QTableWidgetItem(entry.ioIndex));
    table->setCellWidget(row, 2, makeDirectionCombo(table, entry.output));
}

QString indicatorStyle(const QString& color)
{
    return QStringLiteral(
        "QPushButton { min-width: 18px; max-width: 18px; min-height: 18px; max-height: 18px;"
        " border-radius: 9px; border: 1px solid #555; background: %1; padding: 0; }"
        "QPushButton:hover { border: 1px solid #111; }").arg(color);
}

} // namespace

ProcessDeviceManagerDialog::ProcessDeviceManagerDialog(ProcessDeviceManager& manager,
                                                       lcnc::ProcessSettings& settings,
                                                       ProcessDeviceKind initialKind,
                                                       QWidget* parent)
    : QDialog(parent)
    , m_manager(manager)
    , m_settings(settings)
{
    m_machineConfig = lcnc::Kernel::current().service<lcnc::MachineConfigurationService>();
    setWindowTitle(tr("外设管理"));
    resize(980, 640);
    buildUi();
    loadEditorValues();
    refreshPages();
    if (m_roleList)
        m_roleList->setCurrentRow(initialKind == ProcessDeviceKind::Laser ? 1 : 0);
    m_ioRefreshTimer = new QTimer(this);
    m_ioRefreshTimer->setInterval(500);
    connect(m_ioRefreshTimer, &QTimer::timeout,
            this, &ProcessDeviceManagerDialog::refreshIoStatesAsync);
    m_ioRefreshTimer->start();
    refreshIoStatesAsync();
}

void ProcessDeviceManagerDialog::buildUi()
{
    auto* root = new QVBoxLayout(this);
    auto* body = new QHBoxLayout();

    m_roleList = new QListWidget(this);
    m_roleList->setMinimumWidth(220);
    m_roleList->setMaximumWidth(260);
    m_roleList->addItem(tr("运动控制器"));
    m_roleList->addItem(tr("激光器"));
    connect(m_roleList, &QListWidget::currentRowChanged,
            this, &ProcessDeviceManagerDialog::switchCurrentPage);

    m_pages = new QStackedWidget(this);
    m_pages->addWidget(buildMotionControllerPage());
    m_pages->addWidget(buildLaserPage());

    body->addWidget(m_roleList);
    body->addWidget(m_pages, 1);
    root->addLayout(body, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Apply | QDialogButtonBox::Cancel, this);
    connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked,
            this, &ProcessDeviceManagerDialog::applySettings);
    connect(buttons, &QDialogButtonBox::accepted, this, [this] {
        applySettings();
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);
    m_roleList->setCurrentRow(0);
}

QWidget* ProcessDeviceManagerDialog::buildMotionControllerPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    auto* top = new QGroupBox(tr("运动控制核心"), page);
    auto* form = new QFormLayout(top);
    m_motionModelCombo = new QComboBox(top);
    for (const auto& descriptor : m_manager.descriptors(ProcessDeviceKind::MotionController)) {
        m_motionModelCombo->addItem(descriptor.displayName, descriptor.name);
        const int row = m_motionModelCombo->count() - 1;
        m_motionModelCombo->setItemData(row,
                                        processDeviceAvailabilityName(descriptor.availability),
                                        Qt::ToolTipRole);
    }
    const int motionIndex = m_motionModelCombo->findData(m_settings.motionControllerName());
    if (motionIndex >= 0)
        m_motionModelCombo->setCurrentIndex(motionIndex);
    form->addRow(tr("型号"), m_motionModelCombo);
    layout->addWidget(top);

    auto* tabs = new QTabWidget(page);
    tabs->addTab(buildStatusPage(ProcessDeviceKind::MotionController), tr("状态"));
    tabs->addTab(buildAxisConfigurationPage(), tr("轴系配置"));
    tabs->addTab(buildLegacyUiPage(QStringLiteral("Setting_IOIndex"), QStringLiteral(":/process/setting/Setting_IOIndex.ui")), tr("IO索引"));
    tabs->addTab(buildCustomIoPage(false), tr("数字IO"));
    tabs->addTab(buildCustomIoPage(true), tr("模拟IO"));
    tabs->addTab(buildMotionDebugPage(), tr("运动调试"));
    tabs->addTab(buildIoDebugPage(), tr("IO调试"));
    layout->addWidget(tabs, 1);
    return page;
}

QWidget* ProcessDeviceManagerDialog::buildLaserPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    auto* top = new QGroupBox(tr("激光器"), page);
    auto* form = new QFormLayout(top);
    m_laserModelCombo = new QComboBox(top);
    for (const auto& descriptor : m_manager.descriptors(ProcessDeviceKind::Laser)) {
        m_laserModelCombo->addItem(descriptor.displayName, descriptor.name);
        const int row = m_laserModelCombo->count() - 1;
        m_laserModelCombo->setItemData(row,
                                       processDeviceAvailabilityName(descriptor.availability),
                                       Qt::ToolTipRole);
    }
    const int laserIndex = m_laserModelCombo->findData(m_settings.laserDeviceName());
    if (laserIndex >= 0)
        m_laserModelCombo->setCurrentIndex(laserIndex);
    form->addRow(tr("型号"), m_laserModelCombo);
    layout->addWidget(top);

    auto* tabs = new QTabWidget(page);
    tabs->addTab(buildStatusPage(ProcessDeviceKind::Laser), tr("状态"));
    tabs->addTab(buildLegacyUiPage(QStringLiteral("Setting_Laser"), QStringLiteral(":/process/setting/Setting_Laser.ui")), tr("参数"));
    tabs->addTab(buildLaserDebugPage(), tr("调试"));
    layout->addWidget(tabs, 1);
    return page;
}

QWidget* ProcessDeviceManagerDialog::buildLegacyUiPage(const QString& pageId, const QString& resourcePath)
{
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        scroll->setWidget(valueLabel(tr("无法加载 UI: %1").arg(resourcePath), scroll));
        return scroll;
    }

    DeviceUiLoader loader;
    QWidget* widget = loader.load(&file, scroll);
    if (!widget) {
        scroll->setWidget(valueLabel(loader.errorString(), scroll));
        return scroll;
    }
    registerLegacyEditors(pageId, widget);
    scroll->setWidget(widget);
    return scroll;
}

QWidget* ProcessDeviceManagerDialog::buildStatusPage(ProcessDeviceKind kind)
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    appendDeviceStatus(layout, kind);
    layout->addStretch(1);
    return page;
}

QWidget* ProcessDeviceManagerDialog::buildAxisConfigurationPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    m_axisConfigTable = new QTableWidget(page);
    m_axisConfigTable->setColumnCount(10);
    m_axisConfigTable->setHorizontalHeaderLabels({
        tr("轴名"), tr("控制器索引"), tr("归零索引"), tr("低速"), tr("中速"),
        tr("高速"), tr("加速度"), tr("加加速度"), tr("负限位"), tr("正限位")
    });
    m_axisConfigTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_axisConfigTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_axisConfigTable->verticalHeader()->setVisible(false);
    m_axisConfigTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_axisConfigTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_axisConfigTable->setAlternatingRowColors(true);
    populateAxisConfigTable();

    layout->addWidget(m_axisConfigTable, 1);
    return page;
}

QWidget* ProcessDeviceManagerDialog::buildCustomIoPage(bool analog)
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    auto* table = new QTableWidget(page);
    table->setColumnCount(3);
    table->setHorizontalHeaderLabels({ tr("IO名"), tr("IO索引"), tr("方向") });
    table->horizontalHeader()->setStretchLastSection(false);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    table->verticalHeader()->setVisible(false);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::ExtendedSelection);
    table->setAlternatingRowColors(true);
    populateCustomIoTable(table, analog ? m_settings.customAnalogIoTable() : m_settings.customDigitalIoTable());

    auto* buttonRow = new QHBoxLayout();
    auto* addButton = new QPushButton(tr("添加"), page);
    auto* removeButton = new QPushButton(tr("删除"), page);
    buttonRow->addWidget(addButton);
    buttonRow->addWidget(removeButton);
    buttonRow->addStretch(1);

    connect(addButton, &QPushButton::clicked, this, [table, analog] {
        lcnc::ProcessIoTableEntry entry;
        entry.name = analog ? QStringLiteral("AnalogIO") : QStringLiteral("DigitalIO");
        entry.ioIndex = analog ? QStringLiteral("AI0") : QStringLiteral("DI0");
        entry.output = false;
        insertIoTableRow(table, entry);
        table->setCurrentCell(table->rowCount() - 1, 0);
        table->editItem(table->item(table->rowCount() - 1, 0));
    });
    connect(removeButton, &QPushButton::clicked, this, [table] {
        QList<int> rows;
        for (const QModelIndex& index : table->selectionModel()->selectedRows())
            rows.append(index.row());
        std::sort(rows.begin(), rows.end(), std::greater<int>());
        for (int row : rows)
            table->removeRow(row);
    });

    layout->addWidget(table, 1);
    layout->addLayout(buttonRow);
    if (analog)
        m_customAnalogIoTable = table;
    else
        m_customDigitalIoTable = table;
    return page;
}

QWidget* ProcessDeviceManagerDialog::buildMotionDebugPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(valueLabel(tr("运动控制器调试动作沿用右侧运行控制面板；此处保留控制器状态、型号和 IO 调试。"), page));
    layout->addStretch(1);
    return page;
}

QWidget* ProcessDeviceManagerDialog::buildIoDebugPage()
{
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    auto* page = new QWidget(scroll);
    auto* layout = new QVBoxLayout(page);

    const QList<ProcessIoConsoleItem> items = ioConsoleItems();
    auto addSection = [&](const QString& title, bool analog, bool output) {
        auto* group = new QGroupBox(title, page);
        auto* grid = new QGridLayout(group);
        int row = 0;
        for (const ProcessIoConsoleItem& item : items) {
            if (item.analog != analog || item.output != output)
                continue;
            auto* name = valueLabel(QStringLiteral("%1  [%2]").arg(item.name, item.ioIndex), group);
            grid->addWidget(name, row, 0);
            if (analog) {
                auto* value = valueLabel(QStringLiteral("--"), group);
                value->setMinimumWidth(80);
                m_analogValueLabels.insert(item.key, value);
                grid->addWidget(value, row, 1);
                if (output) {
                    auto* spin = new QDoubleSpinBox(group);
                    spin->setRange(-1000000.0, 1000000.0);
                    spin->setDecimals(3);
                    auto* write = new QPushButton(tr("写入"), group);
                    connect(write, &QPushButton::clicked, this, [this, item, spin] {
                        IProcessIo* io = m_manager.processIo();
                        if (!io) {
                            showError(tr("IO 控制器未创建"));
                            return;
                        }
                        QString error;
                        if (!io->setAnalogOutput(item.ioIndex, spin->value(), &error))
                            showError(error);
                        refreshIoStatesAsync();
                    });
                    auto* rowLayout = new QHBoxLayout();
                    rowLayout->addWidget(spin);
                    rowLayout->addWidget(write);
                    grid->addLayout(rowLayout, row, 2);
                }
            } else {
                auto* indicator = new QPushButton(group);
                indicator->setToolTip(output ? tr("点击切换输出电平") : tr("输入电平状态"));
                indicator->setCursor(output ? Qt::PointingHandCursor : Qt::ArrowCursor);
                indicator->setProperty("ioValue", false);
                setDigitalIndicator(indicator, false, false);
                m_digitalIndicators.insert(item.key, indicator);
                if (output) {
                    connect(indicator, &QPushButton::clicked, this, [this, item, indicator] {
                        IProcessIo* io = m_manager.processIo();
                        if (!io) {
                            showError(tr("IO 控制器未创建"));
                            return;
                        }
                        const bool nextValue = !indicator->property("ioValue").toBool();
                        QString error;
                        if (!io->setDigitalOutput(item.ioIndex, nextValue, &error)) {
                            showError(error);
                            return;
                        }
                        setDigitalIndicator(indicator, nextValue);
                        refreshIoStatesAsync();
                    });
                }
                grid->addWidget(indicator, row, 1);
            }
            ++row;
        }
        if (row == 0)
            group->hide();
        else
            layout->addWidget(group);
    };

    addSection(tr("数字输入"), false, false);
    addSection(tr("数字输出"), false, true);
    addSection(tr("模拟输入"), true, false);
    addSection(tr("模拟输出"), true, true);

    if (layout->count() == 0)
        layout->addWidget(valueLabel(tr("暂无可调试 IO，请先在 IO索引或数字/模拟 IO 表中配置。"), page));
    layout->addStretch(1);
    scroll->setWidget(page);
    return scroll;
}

QWidget* ProcessDeviceManagerDialog::buildLaserDebugPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    auto* laser = m_manager.laserDevice();
    auto* group = new QGroupBox(tr("激光调试"), page);
    auto* form = new QFormLayout(group);
    auto* energy = new QDoubleSpinBox(group);
    energy->setRange(0.0, 1000000.0);
    energy->setDecimals(3);
    energy->setValue(laser ? laser->energy() : 0.0);
    auto* frequency = new QDoubleSpinBox(group);
    frequency->setRange(0.0, 1000000.0);
    frequency->setDecimals(3);
    frequency->setValue(laser ? laser->frequency() : 0.0);
    auto* pulse = new QDoubleSpinBox(group);
    pulse->setRange(0.0, 1000000.0);
    pulse->setDecimals(3);
    pulse->setValue(laser ? laser->pulseWidth() : 0.0);
    form->addRow(tr("能量"), energy);
    form->addRow(tr("频率"), frequency);
    form->addRow(tr("脉宽"), pulse);

    auto* row = new QHBoxLayout();
    auto* apply = new QPushButton(tr("下发"), group);
    auto* laserOn = new QPushButton(tr("开光"), group);
    auto* laserOff = new QPushButton(tr("关光"), group);
    auto* aimingOn = new QPushButton(tr("指示光开"), group);
    auto* aimingOff = new QPushButton(tr("指示光关"), group);
    row->addWidget(apply);
    row->addWidget(laserOn);
    row->addWidget(laserOff);
    row->addWidget(aimingOn);
    row->addWidget(aimingOff);
    form->addRow(row);

    auto handle = [this](bool ok, const QString& error) {
        if (!ok)
            showError(error);
        refreshPages();
    };
    connect(apply, &QPushButton::clicked, this, [this, laser, energy, frequency, pulse, handle] {
        if (!laser) {
            showError(tr("激光器未创建"));
            return;
        }
        QString error;
        const bool ok = laser->setEnergy(energy->value(), &error)
            && laser->setFrequency(frequency->value(), &error)
            && laser->setPulseWidth(pulse->value(), &error);
        handle(ok, error);
    });
    connect(laserOn, &QPushButton::clicked, this, [laser, handle] {
        QString error;
        handle(laser && laser->startLaser(&error), error);
    });
    connect(laserOff, &QPushButton::clicked, this, [laser, handle] {
        QString error;
        handle(laser && laser->stopLaser(&error), error);
    });
    connect(aimingOn, &QPushButton::clicked, this, [laser, handle] {
        QString error;
        handle(laser && laser->startAiming(&error), error);
    });
    connect(aimingOff, &QPushButton::clicked, this, [laser, handle] {
        QString error;
        handle(laser && laser->stopAiming(&error), error);
    });

    layout->addWidget(group);
    layout->addStretch(1);
    return page;
}

void ProcessDeviceManagerDialog::registerLegacyEditors(const QString& pageId, QWidget* root)
{
    if (!root)
        return;
    const auto widgets = root->findChildren<QWidget*>();
    for (QWidget* widget : widgets) {
        if (!isEditableWidget(widget))
            continue;
        const QString key = QStringLiteral("%1.%2").arg(pageId, widget->objectName().trimmed());
        widget->setProperty("uiSettingKey", key);
        m_legacyEditors.insert(key, widget);
    }
}

void ProcessDeviceManagerDialog::loadEditorValues()
{
    for (auto it = m_legacyEditors.cbegin(); it != m_legacyEditors.cend(); ++it) {
        if (!m_settings.uiSettingValues().contains(it.key()))
            continue;
        setEditorValue(it.value(), m_settings.uiSettingValue(it.key()));
    }
}

void ProcessDeviceManagerDialog::collectEditorValues(QMap<QString, QString>& values) const
{
    for (auto it = m_legacyEditors.cbegin(); it != m_legacyEditors.cend(); ++it)
        values.insert(it.key(), editorValue(it.value()));
}

void ProcessDeviceManagerDialog::appendDeviceStatus(QVBoxLayout* layout, ProcessDeviceKind kind)
{
    bool found = false;
    const ProcessDeviceSession session = roleSession(kind, &found);
    auto* group = new QGroupBox(tr("状态监控"), this);
    auto* form = new QFormLayout(group);
    if (!found) {
        form->addRow(tr("状态"), valueLabel(tr("未配置"), group));
        layout->addWidget(group);
        return;
    }
    form->addRow(tr("型号"), valueLabel(session.displayName, group));
    form->addRow(tr("状态"), valueLabel(processDeviceConnectionStateName(session.connectionState), group));
    if (!session.lastError.isEmpty())
        form->addRow(tr("异常"), valueLabel(session.lastError, group));
    if (IProcessDevice* device = m_manager.processDevice(session.instanceId)) {
        form->addRow(tr("能力"), valueLabel(device->capabilities().join(QStringLiteral(", ")), group));
        for (const auto& item : device->statusItems())
            form->addRow(item.name, valueLabel(item.value, group));
    }
    if (kind == ProcessDeviceKind::MotionController) {
        if (IProcessIo* io = m_manager.processIo()) {
            form->addRow(tr("IO状态"), valueLabel(processDeviceConnectionStateName(io->connectionState()), group));
            if (!io->lastError().isEmpty())
                form->addRow(tr("IO异常"), valueLabel(io->lastError(), group));
            for (const auto& item : io->statusItems())
                form->addRow(QStringLiteral("IO.%1").arg(item.name), valueLabel(item.value, group));
        }
    }
    layout->addWidget(group);
}

ProcessDeviceSession ProcessDeviceManagerDialog::roleSession(ProcessDeviceKind kind, bool* found) const
{
    const ProcessDeviceRole role = kind == ProcessDeviceKind::MotionController
        ? ProcessDeviceRole::ActiveMotion
        : ProcessDeviceRole::ActiveLaser;
    for (const ProcessDeviceSession& session : m_manager.deviceSessions()) {
        if (session.kind == kind && session.role == role) {
            if (found)
                *found = true;
            return session;
        }
    }
    if (found)
        *found = false;
    return {};
}

void ProcessDeviceManagerDialog::populateAxisConfigTable()
{
    if (!m_axisConfigTable)
        return;
    m_axisConfigTable->setRowCount(0);
    const QVector<lcnc::MachineAxisRuntimeConfig> configs = m_machineConfig
        ? m_machineConfig->axisConfigurations()
        : QVector<lcnc::MachineAxisRuntimeConfig>{};
    for (const lcnc::MachineAxisRuntimeConfig& config : configs)
        insertAxisConfigRow(m_axisConfigTable, config);
}

QVector<lcnc::MachineAxisRuntimeConfig> ProcessDeviceManagerDialog::readAxisConfigTable() const
{
    QVector<lcnc::MachineAxisRuntimeConfig> configs;
    if (!m_axisConfigTable)
        return configs;

    auto itemText = [this](int row, int column) {
        QTableWidgetItem* item = m_axisConfigTable->item(row, column);
        return item ? item->text().trimmed() : QString();
    };
    auto itemDouble = [&](int row, int column, double fallback) {
        bool ok = false;
        const double value = itemText(row, column).toDouble(&ok);
        return ok ? value : fallback;
    };
    auto itemInt = [&](int row, int column, int fallback) {
        bool ok = false;
        const int value = itemText(row, column).toInt(&ok);
        return ok ? value : fallback;
    };

    for (int row = 0; row < m_axisConfigTable->rowCount(); ++row) {
        const QString name = itemText(row, 0).toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        lcnc::MachineAxisRuntimeConfig config;
        config.axis.name = name;
        config.controllerIndex = itemInt(row, 1, row);
        config.homeIndex = itemInt(row, 2, config.controllerIndex);
        config.lowSpeed = itemDouble(row, 3, 1.0);
        config.mediumSpeed = itemDouble(row, 4, 10.0);
        config.highSpeed = itemDouble(row, 5, 50.0);
        config.acceleration = itemDouble(row, 6, 200.0);
        config.jerk = itemDouble(row, 7, 0.0);
        config.axis.minVal = itemDouble(row, 8, -300.0);
        config.axis.maxVal = itemDouble(row, 9, 300.0);
        configs.append(config);
    }
    return configs;
}

void ProcessDeviceManagerDialog::populateCustomIoTable(QTableWidget* table,
                                                       const QVector<lcnc::ProcessIoTableEntry>& entries) const
{
    if (!table)
        return;
    table->setRowCount(0);
    for (const lcnc::ProcessIoTableEntry& entry : entries)
        insertIoTableRow(table, entry);
}

QVector<lcnc::ProcessIoTableEntry> ProcessDeviceManagerDialog::readCustomIoTable(QTableWidget* table) const
{
    QVector<lcnc::ProcessIoTableEntry> entries;
    if (!table)
        return entries;
    for (int row = 0; row < table->rowCount(); ++row) {
        lcnc::ProcessIoTableEntry entry;
        if (auto* name = table->item(row, 0))
            entry.name = name->text().trimmed();
        if (auto* index = table->item(row, 1))
            entry.ioIndex = index->text().trimmed();
        if (auto* combo = qobject_cast<QComboBox*>(table->cellWidget(row, 2)))
            entry.output = combo->currentData().toBool();
        if (!entry.name.isEmpty() && !entry.ioIndex.isEmpty())
            entries.append(entry);
    }
    return entries;
}

QList<ProcessIoConsoleItem> ProcessDeviceManagerDialog::ioConsoleItems() const
{
    QList<ProcessIoConsoleItem> items;
    QSet<QString> seen;
    auto appendItem = [&](QString name, QString ioIndex, bool analog, bool output) {
        name = name.trimmed();
        ioIndex = ioIndex.trimmed();
        if (name.isEmpty() || ioIndex.isEmpty())
            return;
        ProcessIoConsoleItem item;
        item.name = name;
        item.ioIndex = ioIndex;
        item.analog = analog;
        item.output = output;
        item.key = QStringLiteral("%1:%2:%3:%4")
            .arg(analog ? QStringLiteral("A") : QStringLiteral("D"),
                 output ? QStringLiteral("O") : QStringLiteral("I"),
                 name,
                 ioIndex);
        if (seen.contains(item.key))
            return;
        seen.insert(item.key);
        items.append(item);
    };

    const QMap<QString, QString> values = m_settings.uiSettingValues();
    const QString presetPrefix = QStringLiteral("Setting_IOIndex.");
    for (auto it = values.cbegin(); it != values.cend(); ++it) {
        const QString key = it.key();
        if (!key.startsWith(presetPrefix))
            continue;
        const QString field = key.mid(presetPrefix.size());
        if (field.startsWith(QStringLiteral("lineEdit_DigitalOUT_")))
            appendItem(ioNameFromLegacyField(field, QStringLiteral("lineEdit_DigitalOUT_")), it.value(), false, true);
        else if (field.startsWith(QStringLiteral("lineEdit_DigitalIN_")))
            appendItem(ioNameFromLegacyField(field, QStringLiteral("lineEdit_DigitalIN_")), it.value(), false, false);
        else if (field.startsWith(QStringLiteral("lineEdit_AnalogOUT_")))
            appendItem(ioNameFromLegacyField(field, QStringLiteral("lineEdit_AnalogOUT_")), it.value(), true, true);
        else if (field.startsWith(QStringLiteral("lineEdit_AnalogIN_")))
            appendItem(ioNameFromLegacyField(field, QStringLiteral("lineEdit_AnalogIN_")), it.value(), true, false);
    }

    for (const lcnc::ProcessIoTableEntry& entry : m_settings.customDigitalIoTable())
        appendItem(entry.name, entry.ioIndex, false, entry.output);
    for (const lcnc::ProcessIoTableEntry& entry : m_settings.customAnalogIoTable())
        appendItem(entry.name, entry.ioIndex, true, entry.output);
    return items;
}

void ProcessDeviceManagerDialog::refreshIoStatesAsync()
{
    if (m_ioRefreshInFlight || (m_digitalIndicators.isEmpty() && m_analogValueLabels.isEmpty()))
        return;
    IProcessIo* io = m_manager.processIo();
    if (!io)
        return;

    const QList<ProcessIoConsoleItem> items = ioConsoleItems();
    m_ioRefreshInFlight = true;
    auto* watcher = new QFutureWatcher<ProcessIoStateSnapshot>(this);
    connect(watcher, &QFutureWatcher<ProcessIoStateSnapshot>::finished, this, [this, watcher] {
        const ProcessIoStateSnapshot snapshot = watcher->result();
        watcher->deleteLater();
        m_ioRefreshInFlight = false;
        applyIoStateSnapshot(snapshot);
    });
    watcher->setFuture(QtConcurrent::run([io, items] {
        ProcessIoStateSnapshot snapshot;
        for (const ProcessIoConsoleItem& item : items) {
            QString error;
            if (item.analog) {
                double value = 0.0;
                if (io->analogInput(item.ioIndex, &value, &error))
                    snapshot.analogValues.insert(item.key, value);
                else if (!error.isEmpty())
                    snapshot.errors.append(QStringLiteral("%1: %2").arg(item.name, error));
            } else {
                bool value = false;
                if (io->digitalInput(item.ioIndex, &value, &error))
                    snapshot.digitalValues.insert(item.key, value);
                else if (!error.isEmpty())
                    snapshot.errors.append(QStringLiteral("%1: %2").arg(item.name, error));
            }
        }
        return snapshot;
    }));
}

void ProcessDeviceManagerDialog::applyIoStateSnapshot(const ProcessIoStateSnapshot& snapshot)
{
    for (auto it = m_digitalIndicators.begin(); it != m_digitalIndicators.end(); ++it) {
        if (snapshot.digitalValues.contains(it.key()))
            setDigitalIndicator(it.value(), snapshot.digitalValues.value(it.key()));
        else
            setDigitalIndicator(it.value(), false, false);
    }
    for (auto it = m_analogValueLabels.begin(); it != m_analogValueLabels.end(); ++it) {
        QLabel* label = it.value();
        if (!label)
            continue;
        if (snapshot.analogValues.contains(it.key()))
            label->setText(QString::number(snapshot.analogValues.value(it.key()), 'f', 3));
        else
            label->setText(QStringLiteral("--"));
    }
}

void ProcessDeviceManagerDialog::setDigitalIndicator(QPushButton* indicator, bool high, bool valid)
{
    if (!indicator)
        return;
    indicator->setProperty("ioValue", high);
    indicator->setStyleSheet(indicatorStyle(valid
        ? (high ? QStringLiteral("#1f9d55") : QStringLiteral("#d64545"))
        : QStringLiteral("#9ca3af")));
}

void ProcessDeviceManagerDialog::refreshPages()
{
    const int row = m_roleList ? m_roleList->currentRow() : 0;
    m_axisConfigTable = nullptr;
    m_customDigitalIoTable = nullptr;
    m_customAnalogIoTable = nullptr;
    m_digitalIndicators.clear();
    m_analogValueLabels.clear();
    while (m_pages->count() > 0) {
        QWidget* widget = m_pages->widget(0);
        m_pages->removeWidget(widget);
        widget->deleteLater();
    }
    m_legacyEditors.clear();
    m_pages->addWidget(buildMotionControllerPage());
    m_pages->addWidget(buildLaserPage());
    loadEditorValues();
    const int targetRow = row < 0 ? 0 : row;
    if (m_roleList)
        m_roleList->setCurrentRow(targetRow);
    switchCurrentPage(targetRow);
    refreshIoStatesAsync();
}

void ProcessDeviceManagerDialog::applySettings()
{
    if (m_motionModelCombo)
        m_settings.setMotionControllerName(m_motionModelCombo->currentData().toString());
    if (m_laserModelCombo)
        m_settings.setLaserDeviceName(m_laserModelCombo->currentData().toString());
    if (m_machineConfig)
        m_machineConfig->setAxisHardwareConfigurations(readAxisConfigTable());
    m_settings.setCustomDigitalIoTable(readCustomIoTable(m_customDigitalIoTable));
    m_settings.setCustomAnalogIoTable(readCustomIoTable(m_customAnalogIoTable));
    QMap<QString, QString> values = m_settings.uiSettingValues();
    collectEditorValues(values);
    m_settings.setUiSettingValues(values);
    m_manager.syncFromSettings(m_settings);
    emit settingsApplied();
    refreshPages();
}

void ProcessDeviceManagerDialog::switchCurrentPage(int row)
{
    if (row >= 0 && row < m_pages->count())
        m_pages->setCurrentIndex(row);
}

void ProcessDeviceManagerDialog::showError(const QString& message)
{
    QMessageBox::warning(this, tr("外设管理"), message.isEmpty() ? tr("操作失败") : message);
}

} // namespace lcnc::process
