#include "core/kernel/service_registry.h"

#include <cassert>
#include <memory>

namespace {

class TestService final : public lcnc::IService {
  public:
    explicit TestService(bool& destroyed) : m_destroyed(destroyed) {}

    ~TestService() override {
        m_destroyed = true;
    }

  private:
    bool& m_destroyed;
};

} // namespace

int main() {
    lcnc::ServiceRegistry registry;

    bool borrowedDestroyed = false;
    {
        TestService borrowed(borrowedDestroyed);
        assert(registry.registerBorrowedService<TestService>(borrowed));
        auto lookup = registry.getService<TestService>();
        assert(lookup.get() == &borrowed);
        registry.clear();
        lookup.reset();
        assert(!borrowedDestroyed);
    }
    assert(borrowedDestroyed);

    bool ownedDestroyed = false;
    auto owned = std::make_shared<TestService>(ownedDestroyed);
    assert(registry.registerService<TestService>(owned));
    owned.reset();
    assert(!ownedDestroyed);
    registry.clear();
    assert(ownedDestroyed);
    return 0;
}
