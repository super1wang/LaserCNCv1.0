#include "modules/cam/ui/collision/widget_collision_detection_panel.h"

#include "core/kernel/kernel.h"
#include "modules/cam/contracts/i_cam_collision_configuration_provider.h"

#include <QCheckBox>
#include <QGroupBox>
#include <QLabel>
#include <QLayoutItem>
#include <QMessageBox>
#include <QTimer>
#include <QVBoxLayout>

namespace lcnc::cam::ui {

namespace {

void clearLayout(QVBoxLayout* layout)
{
    while (layout && layout->count() > 0) {
        QLayoutItem* item = layout->takeAt(0);
        delete item->widget();
        delete item;
    }
}

} // namespace

WidgetCollisionDetectionPanel::WidgetCollisionDetectionPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(8);

    m_enabled = new QCheckBox(tr("Enable collision detection"), this);
    m_enabled->setToolTip(tr("Enable offline and full-machine collision verification for the current machine configuration"));
    root->addWidget(m_enabled);

    m_status = new QLabel(this);
    m_status->setWordWrap(true);
    m_status->setStyleSheet(QStringLiteral("color:#666;"));
    root->addWidget(m_status);

    m_activeGroup = new QGroupBox(tr("Active sources"), this);
    m_activeLayout = new QVBoxLayout(m_activeGroup);
    root->addWidget(m_activeGroup);

    m_passiveGroup = new QGroupBox(tr("Passive sources"), this);
    m_passiveLayout = new QVBoxLayout(m_passiveGroup);
    root->addWidget(m_passiveGroup);
    root->addStretch();

    connect(m_enabled, &QCheckBox::toggled, this, [this](bool enabled) {
        if (m_syncing) return;
        if (auto* provider = lcnc::Kernel::current().service<ICamCollisionConfigurationProvider>()) {
            QString error;
            if (!provider->setCollisionDetectionEnabled(enabled, &error)
                && !error.isEmpty()) {
                // 中文翻译：无法启用碰撞检测
                QMessageBox::warning(this, tr("Unable to enable collision detection"),
                                     error);
            }
        }
        refresh();
    });
    refreshNow();
}

void WidgetCollisionDetectionPanel::refresh()
{
    // Source checkboxes are rebuilt by deleting their widgets. A configuration
    // change is emitted synchronously from a checkbox's toggled() handler, so
    // rebuilding here would delete that sender before signal delivery returns.
    // Queue one coalesced refresh instead; it also makes rapid multi-check
    // changes stable without redundant layout churn.
    // 中文翻译：配置变更会在 toggled() 内同步发出，必须延后重建，避免删除正在发信号的勾选框。
    if (m_refreshQueued)
        return;
    m_refreshQueued = true;
    QTimer::singleShot(0, this, [this] {
        m_refreshQueued = false;
        refreshNow();
    });
}

void WidgetCollisionDetectionPanel::refreshNow()
{
    m_syncing = true;
    rebuildSources();
    m_syncing = false;
}

void WidgetCollisionDetectionPanel::rebuildSources()
{
    auto* provider = lcnc::Kernel::current().service<ICamCollisionConfigurationProvider>();
    clearLayout(m_activeLayout);
    clearLayout(m_passiveLayout);
    if (!provider) {
        m_enabled->setChecked(false);
        m_enabled->setEnabled(false);
        m_status->setText(tr("Collision configuration service is unavailable."));
        return;
    }

    const CollisionConfigurationSnapshot snapshot = provider->collisionConfiguration();
    m_enabled->setChecked(snapshot.enabled);
    m_enabled->setEnabled(snapshot.enabled
                          || (snapshot.activationAvailable && snapshot.valid));
    if (!snapshot.activationAvailable) {
        m_status->setText(snapshot.activationFailureReason);
    } else if (!snapshot.valid) {
        m_status->setText(tr("Select at least one available active source and one available passive source before enabling detection."));
    } else if (snapshot.enabled) {
        m_status->setText(tr("Collision detection is enabled. Machine roles are derived from the immutable package; the workpiece is the only variable."));
    } else {
        // 中文翻译：碰撞验证已禁用；轨迹可用于调试，但未获得碰撞安全认证。
        m_status->setText(tr("Collision verification is disabled; motion remains available for commissioning but is not collision-certified."));
    }
    if (snapshot.unassignedMachineBodyCount > 0)
        m_status->setText(m_status->text() + QLatin1Char('\n')
            + tr("%1 machine parts are not assigned to an axis and are excluded.")
                  .arg(snapshot.unassignedMachineBodyCount));

    const auto add = [this, provider, snapshot](const CollisionSourceDescriptor& source,
                                                  bool active, QVBoxLayout* layout) {
        if ((active && !source.activeCandidate) || (!active && !source.passiveCandidate))
            return;
        auto* check = new QCheckBox(
            tr("%1 (%2 parts)").arg(source.displayName).arg(source.bodyCount),
            active ? static_cast<QWidget*>(m_activeGroup) : static_cast<QWidget*>(m_passiveGroup));
        check->setChecked((active ? snapshot.activeSources : snapshot.passiveSources).contains(source.id));
        check->setEnabled(snapshot.sourceSelectionMutable && source.available);
        if (!snapshot.sourceSelectionMutable)
            check->setToolTip(tr("Collision roles are fixed by the machine assembly and workpiece mount chain."));
        if (!source.available)
            check->setToolTip(tr("No collision geometry is available for this source."));
        connect(check, &QCheckBox::toggled, this, [this, provider, source, active](bool checked) {
            if (m_syncing) return;
            const CollisionConfigurationSnapshot current = provider->collisionConfiguration();
            QSet<QString> nextActive = current.activeSources;
            QSet<QString> nextPassive = current.passiveSources;
            QSet<QString>& target = active ? nextActive : nextPassive;
            if (checked) target.insert(source.id); else target.remove(source.id);
            provider->setCollisionSources(nextActive, nextPassive);
            refresh();
        });
        layout->addWidget(check);
    };
    for (const auto& source : snapshot.sources) {
        add(source, true, m_activeLayout);
        add(source, false, m_passiveLayout);
    }
}

} // namespace lcnc::cam::ui
