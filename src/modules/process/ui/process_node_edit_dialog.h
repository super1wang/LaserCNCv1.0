#pragma once

#include "modules/process/workflow/process_node.h"

#include <QDialog>
#include <QMap>

class QCheckBox;
class QDoubleSpinBox;
class QComboBox;
class QLineEdit;
class QSpinBox;
class QStackedWidget;
class QTableWidget;

namespace lcnc::process {

/**
 * @brief Basic editor for workflow nodes until specialized node editors migrate.
 */
class ProcessNodeEditDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ProcessNodeEditDialog(ProcessNode node, QWidget* parent = nullptr);

    ProcessNode node() const;

private:
    QWidget* buildGeneralPage();
    QWidget* buildParameterPage();
    QWidget* buildPluginParameterPage();
    QWidget* buildTypedParameterPage();
    QWidget* buildWaitPage();
    QWidget* buildAxisPage();
    void loadNode();
    void applyNode();
    void loadParameterTable();
    void applyParameterTable();
    void loadTypedParameterEditors();
    void applyTypedParameterEditors();
    QWidget* buildMultiAxisPage();
    void addAxisRow(const QVariantMap& row = {});
    QVariantList axesFromTable() const;
    void loadMultiAxisPage();
    void applyMultiAxisPage();

    ProcessNode m_node;
    QLineEdit* m_nameEdit{nullptr};
    QCheckBox* m_enabledCheck{nullptr};
    QStackedWidget* m_detailStack{nullptr};
    QWidget* m_pluginEditor{nullptr};
    QTableWidget* m_parameterTable{nullptr};
    QSpinBox* m_waitDurationSpin{nullptr};
    QLineEdit* m_axisNameEdit{nullptr};
    QDoubleSpinBox* m_axisPositionSpin{nullptr};
    QTableWidget* m_axesTable{nullptr};
    QComboBox* m_multiModeCombo{nullptr};
    QMap<QString, QLineEdit*> m_textEditors;
    QMap<QString, QComboBox*> m_comboEditors;
    QMap<QString, QDoubleSpinBox*> m_doubleEditors;
    QMap<QString, QSpinBox*> m_intEditors;
    QMap<QString, QCheckBox*> m_boolEditors;
};

} // namespace lcnc::process
