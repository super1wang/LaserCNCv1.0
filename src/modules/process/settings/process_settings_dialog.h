#pragma once

#include "modules/process/settings/process_parameter_registry.h"

#include <QDialog>
#include <QVector>

class QLineEdit;
class QStackedWidget;
class QTableView;
class QTreeView;
class QTreeWidget;
class Service;

namespace lcnc::process {
class ProcessPropertyModel;
class ProcessIoTableModel;
class ProcessSettingsService;

class ProcessSettingsDialog final : public QDialog
{
public:
    ProcessSettingsDialog(ProcessSettingsService* settings, Service* runtime, QWidget* parent = nullptr);

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
    Service* m_runtime{nullptr};
    QTreeWidget* m_objects{nullptr};
    QTreeView* m_properties{nullptr};
    QTableView* m_ioTable{nullptr};
    QStackedWidget* m_editorStack{nullptr};
    QLineEdit* m_search{nullptr};
    ProcessPropertyModel* m_model{nullptr};
    ProcessIoTableModel* m_ioModel{nullptr};
    QVector<ParameterObjectDescriptor> m_objectDescriptors;
};
} // namespace lcnc::process
