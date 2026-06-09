#include "modules/process/Setting/setting_bridge.h"
#include "modules/process/Setting/qg_dlgsetting.h"
#include "Service.h"
#include "core/kernel/kernel.h"
#include "modules/process/process_module.h"

void initProcessService()
{
    auto* mod = lcnc::Kernel::current().service<ProcessModule>();
    if (!mod) return;

    auto* svc = new Service();
    svc->SetMotionControl("Simulator");
    svc->SetLaserDevice("Simulator");
    mod->setService(svc);
}

bool openSettingsDialog()
{
    auto* dlg = QG_dlgSetting::instance();
    if (!dlg) return false;

    auto* mod = lcnc::Kernel::current().service<ProcessModule>();
    if (mod && mod->service())
        dlg->SetService(mod->service());

    dlg->InitSetting();
    return dlg->exec() == QDialog::Accepted;
}
