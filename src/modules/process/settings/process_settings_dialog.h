#pragma once

#include "modules/process/settings/process_parameter_registry.h"

#include <QDialog>
#include <QVector>
#include <functional>

class QLineEdit;
class QAction;
class QStackedWidget;
class QTableView;
class QTreeView;
class QTreeWidget;

namespace lcnc::process {
class ProcessPropertyModel;
class ProcessIoTableModel;
class ProcessSettingsService;
struct ProcessSettingsChangeSet;

class ProcessSettingsDialog final : public QDialog
{
    Q_OBJECT

public:
    using SettingsAppliedHandler = std::function<void(const ProcessSettingsChangeSet&)>;

    ProcessSettingsDialog(ProcessSettingsService* settings,
                          SettingsAppliedHandler settingsApplied,
                          QWidget* parent = nullptr);

private:
    void rebuildObjectTree();
    void showCurrentObject();
    void apply();
    void createTool();
    void copyTool();
    void renameTool();
    void deleteTool();
    QString selectedToolName() const;

    ProcessSettingsService* m_settings{nullptr};
    SettingsAppliedHandler m_settingsApplied;
    QTreeWidget* m_objects{nullptr};
    QTreeView* m_properties{nullptr};
    QTableView* m_ioTable{nullptr};
    QStackedWidget* m_editorStack{nullptr};
    QLineEdit* m_search{nullptr};
    ProcessPropertyModel* m_model{nullptr};
    ProcessIoTableModel* m_ioModel{nullptr};
    QVector<ParameterObjectDescriptor> m_objectDescriptors;
    QAction* m_renameToolAction{nullptr};
    QAction* m_deleteToolAction{nullptr};
};
} // namespace lcnc::process
