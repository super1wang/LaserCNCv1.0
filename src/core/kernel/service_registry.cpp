#include "core/kernel/service_registry.h"

#include "core/logging/logger.h"

namespace lcnc {

void ServiceRegistry::clear()
{
    LCNC_DEBUG(LogCode::Generic,
               "ServiceRegistry::clear() begin (count={})", m_map.size());
    // 反序销毁可避免后注册的服务依赖先注册者。当前用 unordered_map 无序，
    // 假定调用方在 shutdown 期间已停掉所有模块，服务间不再交互。
    m_map.clear();
    LCNC_DEBUG(LogCode::Generic, "ServiceRegistry::clear() end");
}

} // namespace lcnc
