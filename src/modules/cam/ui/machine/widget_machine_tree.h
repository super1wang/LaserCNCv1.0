#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QSet>

class LcncDocument;
class MachineKinematics;

namespace lcnc::cam {
class MachineWorkspace;
}

class WidgetMachineTree : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetMachineTree(QWidget* parent = nullptr);

    void setWorkspace(lcnc::cam::MachineWorkspace* workspace);
    void setVisibilityState(bool machineModelVisible, const QStringList& visibleEntries);

public slots:
    void rebuild();

signals:
    void shapeVisibilityChanged(const QString& entry, bool visible);

private:
    void setupUi();
    QTreeWidgetItem* buildAxisNode(const QString& axisName,
                                    const QStringList& entries,
                                    const QMap<QString, QString>& nameMap);
    QTreeWidgetItem* buildUnassignedNode(const QMap<QString, QString>& nameMap,
                                          const QSet<QString>& placed);

    QTreeWidget*                m_tree{nullptr};
    lcnc::cam::MachineWorkspace* m_workspace{nullptr};
    bool                         m_machineModelVisible{true};
    QSet<QString>                m_visibleEntries;
    bool                         m_rebuilding{false}; ///< 防止 rebuild 期间 itemChanged 递归
};
