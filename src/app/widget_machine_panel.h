#pragma once

#include <QWidget>
#include <QString>

/**
 * @brief Right-panel widget shown when the "准备" tab is active.
 *
 * Provides machine model management controls:
 *  - Load Machine / Workpiece buttons
 *  - Machine name / origin display
 *  - Quick-transform controls (set home position)
 *
 * Phase 1: basic scaffold with buttons.
 * Phase 3+: machine kinematics configuration.
 */
class WidgetMachinePanel : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetMachinePanel(QWidget* parent = nullptr);

signals:
    void loadMachineRequested();
    void loadWorkpieceRequested();
    void setMachineOriginRequested();

private:
    void buildUi();
};
