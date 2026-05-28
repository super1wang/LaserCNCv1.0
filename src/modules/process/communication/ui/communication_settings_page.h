#pragma once

#include "modules/process/communication/communication_endpoint.h"

#include <QWidget>

class QComboBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableView;

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

class CommunicationManager;

class CommunicationSettingsPage : public QWidget
{
    Q_OBJECT
public:
    explicit CommunicationSettingsPage(QWidget* parent = nullptr);

    void loadFromSettings(const lcnc::ProcessSettings& settings);
    void applyToSettings(lcnc::ProcessSettings& settings) const;

private slots:
    void connectCurrentEndpoint();
    void disconnectCurrentEndpoint();
    void sendPayload();
    void updateProtocolFields();

private:
    CommunicationEndpoint endpointFromControls() const;
    void setStateText(CommunicationState state);

    CommunicationManager* m_manager{nullptr};
    QLineEdit* m_deviceIdEdit{nullptr};
    QComboBox* m_protocolCombo{nullptr};
    QLineEdit* m_hostEdit{nullptr};
    QSpinBox* m_portSpin{nullptr};
    QLineEdit* m_pathEdit{nullptr};
    QLineEdit* m_serialPortEdit{nullptr};
    QSpinBox* m_baudRateSpin{nullptr};
    QSpinBox* m_timeoutSpin{nullptr};
    QLabel* m_stateLabel{nullptr};
    QPlainTextEdit* m_payloadEdit{nullptr};
    QTableView* m_logView{nullptr};
    QPushButton* m_connectButton{nullptr};
    QPushButton* m_disconnectButton{nullptr};
    QPushButton* m_sendButton{nullptr};
};

} // namespace lcnc::process