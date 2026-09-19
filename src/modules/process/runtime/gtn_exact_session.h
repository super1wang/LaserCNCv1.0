#pragma once

#include "modules/process/runtime/gtn_exact_plan_lowering.h"

namespace lcnc::process {
bool validateGtnGroupProfile(const PreparedDeviceProgram&, QString*);
QString gtnParameterAudit(const QString& parameter, double requested, double effective,
                         const QString& unit, const GtnLoweringProfile&);

enum class GtnSealResult { Pending, Sealed, Failed };
enum class GtnSessionState { Idle, Filling, Sealing, Sealed, Running, Complete, Faulted };

// All operations run under the existing device queue/coordinator, never a
// second worker. SDK adapter and deterministic fault tests use this same port.
class IGtnExactBackend {
public:
    virtual ~IGtnExactBackend() = default;
    virtual bool admit(const PreparedDeviceProgram&, int ordinal, QString*) = 0;
    virtual bool acquire(const PreparedDeviceProgram&, QString*) = 0;
    virtual bool validateRtcp(const std::array<double, 5>&, const std::array<double, 5>&, QString*) = 0;
    virtual bool append(const GtnEncodedSection&, const GtnExactCommand&, QString*) = 0;
    virtual GtnSealResult seal(QString*) = 0;
    virtual bool start(QString*) = 0;
    virtual bool poll(bool& running, QString*) = 0;
    // Must make outputs safe and confirm stop before releasing axes. A failed
    // release retains ownership. latch=true forbids automatic start recovery.
    virtual bool stopRelease(bool latch, QString*) = 0;
};

class GtnExactSession final {
public:
    explicit GtnExactSession(IGtnExactBackend& backend) : m_backend(backend) {}
    bool prepare(const PreparedDeviceProgram&, int ordinal, const std::function<bool()>& cancelled, QString*);
    bool fill(const PreparedDeviceProgram&, int ordinal, bool& complete, const std::function<bool()>& cancelled, QString*);
    bool start(const PreparedDeviceProgram&, int ordinal, const std::function<bool()>& cancelled, QString*);
    bool poll(const PreparedDeviceProgram&, int ordinal, bool& running, const std::function<bool()>& cancelled, QString*);
    bool abort(QString* error);
    GtnSessionState state() const { return m_state; }
    bool ownsGroup() const { return m_ownsGroup; }
private:
    bool matches(const PreparedDeviceProgram&, int ordinal) const;
    bool fail(const QString&, QString*);
    IGtnExactBackend& m_backend;
    GtnSessionState m_state{GtnSessionState::Idle};
    bool m_ownsGroup{false};
    bool m_cleanupAttempted{false};
    std::shared_ptr<const GtnEncodedProgram> m_program;
    int m_ordinal{0};
    int m_cursor{0};
    int m_sealAttempts{0};
};
} // namespace lcnc::process
