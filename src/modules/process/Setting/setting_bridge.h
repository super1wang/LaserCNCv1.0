#pragma once

/// Initialize the old Service subsystem and attach it to ProcessModule.
/// Called once during startup before the settings dialog is opened.
void initProcessService();

/// Open the unified settings dialog (qg_dlgsetting).
bool openSettingsDialog();
