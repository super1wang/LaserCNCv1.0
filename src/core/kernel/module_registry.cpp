#include "core/kernel/module_registry.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "core/kernel/i_kernel.h"
#include "core/logging/logger.h"

namespace lcnc {

void ModuleRegistry::addModule(std::unique_ptr<IModule> module)
{
    if (!module) {
        LCNC_WARN(LogCode::InternalUnexpectedState,
                  "ModuleRegistry::addModule got null");
        return;
    }
    const auto id = module->info().id;
    LCNC_DEBUG(LogCode::Generic,
               "ModuleRegistry::addModule id='{}'", id.toStdString());
    m_modules.push_back(std::move(module));
}

IModule* ModuleRegistry::find(const QString& id) const
{
    for (const auto& m : m_modules) {
        if (m && m->info().id == id) return m.get();
    }
    return nullptr;
}

bool ModuleRegistry::computeStartupOrder()
{
    LCNC_DEBUG(LogCode::Generic,
               "ModuleRegistry::computeStartupOrder begin (n={})", m_modules.size());

    // 构建 id -> module 的索引；同时校验 id 唯一性。
    std::unordered_map<QString, IModule*> byId;
    byId.reserve(m_modules.size());
    for (const auto& m : m_modules) {
        const auto id = m->info().id;
        if (id.isEmpty()) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "ModuleRegistry: module has empty id, abort");
            return false;
        }
        if (byId.count(id) != 0) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "ModuleRegistry: duplicate module id '{}'", id.toStdString());
            return false;
        }
        byId.emplace(id, m.get());
    }

    // Kahn 拓扑：入度统计 → 入度 0 的节点入队 → 取出 → 后继入度-1。
    std::unordered_map<QString, int>         inDeg;
    std::unordered_map<QString, QStringList> reverseDeps;  // dep id -> 依赖它的 id 列表
    for (const auto& m : m_modules) {
        const auto& info = m->info();
        inDeg[info.id] += 0;  // 确保所有节点都在 inDeg 中
        for (const auto& dep : info.dependencies) {
            if (!byId.count(dep)) {
                LCNC_ERR(LogCode::InternalUnexpectedState,
                         "ModuleRegistry: '{}' depends on missing module '{}'",
                         info.id.toStdString(), dep.toStdString());
                return false;
            }
            inDeg[info.id] += 1;
            reverseDeps[dep].push_back(info.id);
        }
    }

    std::vector<IModule*> order;
    order.reserve(m_modules.size());

    QStringList ready;
    for (const auto& [id, n] : inDeg) {
        if (n == 0) ready.push_back(id);
    }
    // 稳定输出：按 id 排序，避免 unordered_map 不确定性
    std::sort(ready.begin(), ready.end());

    while (!ready.empty()) {
        const QString id = ready.front();
        ready.removeFirst();
        order.push_back(byId.at(id));

        QStringList nextReady;
        for (const auto& succ : reverseDeps[id]) {
            if (--inDeg[succ] == 0) nextReady.push_back(succ);
        }
        std::sort(nextReady.begin(), nextReady.end());
        ready = nextReady + ready;  // 把新就绪的放在前面，仍满足拓扑约束
    }

    if (order.size() != m_modules.size()) {
        LCNC_ERR(LogCode::InternalUnexpectedState,
                 "ModuleRegistry: dependency cycle detected (placed {}/{} modules)",
                 order.size(), m_modules.size());
        return false;
    }

    m_startupOrder = std::move(order);
    LCNC_DEBUG(LogCode::Generic,
               "ModuleRegistry::computeStartupOrder ok (n={})", m_startupOrder.size());
    return true;
}

bool ModuleRegistry::startAll(IKernel& kernel)
{
    LCNC_DEBUG(LogCode::Generic,
               "ModuleRegistry::startAll begin (n={})", m_modules.size());

    if (!computeStartupOrder()) {
        return false;
    }

    const auto stopModuleNoThrow = [](IModule* module, const char* phase) {
        if (!module)
            return;
        const auto id = module->info().id;
        try {
            module->stop();
        } catch (const std::exception& e) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "{}: module '{}' stop threw: {}",
                     phase, id.toStdString(), e.what());
        } catch (...) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "{}: module '{}' stop threw unknown exception",
                     phase, id.toStdString());
        }
    };
    const auto rollbackInitialized = [&stopModuleNoThrow](const std::vector<IModule*>& modules,
                                                           const char* phase) {
        for (auto it = modules.rbegin(); it != modules.rend(); ++it)
            stopModuleNoThrow(*it, phase);
    };

    // ── init 阶段 ───────────────────────────────────────────────────────
    std::vector<IModule*> initialized;
    for (auto* m : m_startupOrder) {
        const auto info = m->info();
        LCNC_INFO(LogCode::Generic,
                  "Module init: '{}' v{}",
                  info.id.toStdString(), info.version.toStdString());
        bool ok = false;
        try {
            ok = m->init(kernel);
        } catch (const std::exception& e) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' init threw: {}",
                     info.id.toStdString(), e.what());
            ok = false;
        } catch (...) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' init threw unknown exception",
                     info.id.toStdString());
            ok = false;
        }
        if (!ok) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' init failed; rolling back",
                     info.id.toStdString());
            // 失败模块本身也可能已分配部分 init 资源，必须先清理它。
            stopModuleNoThrow(m, "Init rollback");
            rollbackInitialized(initialized, "Init rollback");
            return false;
        }
        initialized.push_back(m);
    }

    // ── start 阶段 ──────────────────────────────────────────────────────
    for (auto* m : m_startupOrder) {
        const auto info = m->info();
        LCNC_INFO(LogCode::Generic, "Module start: '{}'", info.id.toStdString());
        bool ok = false;
        try {
            ok = m->start();
        } catch (const std::exception& e) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' start threw: {}",
                     info.id.toStdString(), e.what());
            ok = false;
        } catch (...) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' start threw unknown exception",
                     info.id.toStdString());
            ok = false;
        }
        if (!ok) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' start failed; rolling back already-started",
                     info.id.toStdString());
            // 所有模块都已 init，不能只停已 start 的子集；否则后续析构会
            // 遗留订阅、服务注册或后台资源。
            rollbackInitialized(initialized, "Start rollback");
            m_started.clear();
            return false;
        }
        m_started.push_back(m);
    }

    m_initialized = true;
    LCNC_INFO(LogCode::Generic,
              "ModuleRegistry::startAll done ({} module(s) running)",
              m_started.size());
    return true;
}

void ModuleRegistry::stopAll()
{
    LCNC_DEBUG(LogCode::Generic,
               "ModuleRegistry::stopAll begin (started={})", m_started.size());
    for (auto it = m_started.rbegin(); it != m_started.rend(); ++it) {
        const auto id = (*it)->info().id;
        LCNC_INFO(LogCode::Generic, "Module stop: '{}'", id.toStdString());
        try {
            (*it)->stop();
        } catch (const std::exception& e) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' stop threw: {}",
                     id.toStdString(), e.what());
        } catch (...) {
            LCNC_ERR(LogCode::InternalUnexpectedState,
                     "Module '{}' stop threw unknown exception",
                     id.toStdString());
        }
    }
    m_started.clear();
    m_initialized = false;
    LCNC_DEBUG(LogCode::Generic, "ModuleRegistry::stopAll end");
}

} // namespace lcnc
