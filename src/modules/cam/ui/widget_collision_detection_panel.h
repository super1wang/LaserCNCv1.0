#pragma once

#include <QWidget>

class QCheckBox;
class QGroupBox;
class QLabel;
class QVBoxLayout;

namespace lcnc::cam::ui {

/// CAM-side source selection for offline/full-machine collision verification.
class WidgetCollisionDetectionPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetCollisionDetectionPanel(QWidget* parent = nullptr);

public slots:
    void refresh();

private:
    void refreshNow();
    void rebuildSources();

    QCheckBox* m_enabled{nullptr};
    QGroupBox* m_activeGroup{nullptr};
    QGroupBox* m_passiveGroup{nullptr};
    QVBoxLayout* m_activeLayout{nullptr};
    QVBoxLayout* m_passiveLayout{nullptr};
    QLabel* m_status{nullptr};
    bool m_syncing{false};
    bool m_refreshQueued{false};
};

} // namespace lcnc::cam::ui
