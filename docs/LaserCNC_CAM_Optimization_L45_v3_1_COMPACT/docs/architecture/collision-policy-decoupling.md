# Collision Verification Policy — 轨迹规划解耦 FINAL

## 1. Current Repository Gap

At planning HEAD `4254db59696193963392e2946132cf614c8def26`, `process_cutting_safety.cpp::camExecutionBlockReason()` currently blocks when:

- machine package / Job Overlay are required but not ready;
- `motionPlan.collision.complete == false` or state is Pending;
- collision state blocks execution;
- real-machine motion certificates are absent/ineligible.

Therefore simply adding a `Disabled` config enum is insufficient: **Process preflight must be structurally split into plan/runtime readiness and policy-controlled collision eligibility.**

## 2. Current architecture vs new policy

The current repository architecture is intentionally fail-closed for real production machining. v3 introduces `Disabled` as an explicit commissioning/validation mode requested for staged trajectory development. This is a deliberate architecture-policy extension and must be committed together with documentation/config/UI changes; it is not inferred from current code. Production policy remains `Required` unless a separately approved product decision changes it.

## 3. Policy

```cpp
enum class CollisionVerificationMode : uint8_t {
    Disabled,
    Optional,
    Required
};
```

Use `Optional`, not a name that suggests it is automatically safe. UI may label it “仅观察/不阻断”.

The value is captured into the immutable CAM execution context/FinalMotionPlan. A prepared run does not re-read a mutable UI toggle.

## 4. Readiness Split

### 4.1 Plan / runtime readiness — always enforced

- FinalMotionPlan exists and identity is current;
- all values finite;
- machine kinematics/layout/calibration needed by selected motion are valid;
- physical axis mapping/soft limits valid;
- ControllerMotionMode/capability compatible;
- process phases/fences/IO complete;
- tool/process parameters frozen and valid;
- device connected/status readable/no fault/axes enabled;
- Stop/E-stop/runtime ownership rules valid.

### 4.2 Collision eligibility — policy controlled

Only this layer consults machine geometric resources, collision snapshot and edge/block certificates.

## 5. Disabled

MUST:

- plan/optimizer works without machine STEP;
- Process execution preflight does not require `.lmsi`, Job Overlay, Coal/OCCT collision backend, collision snapshot completeness or certificates;
- no collision job is scheduled solely for execution eligibility;
- no collision backend is instantiated/queried on the Disabled execution path;
- UI/log says `collision verification disabled / not certified`;
- initial approach/travel readiness must also separate path readiness from collision certification rather than using a universal “isExecutable means certified” semantic.

MUST NOT:

- fabricate `CertifiedSafe`;
- set `collision.complete=true` as an empty-proof workaround;
- bypass device/kinematic/limit/RTCP/Stop checks;
- claim production collision safety.

Recommended proof state:

```text
verificationMode = Disabled
proofState = NotRequested
```

## 6. Optional

- attach proof when resources exist;
- show Collision/Unknown/Stale prominently;
- proof does not block current commissioning run;
- collision backend never rewrites motion;
- a prepared run cannot silently promote/demote policy.

If product policy initially exposes only Disabled/Required, `Optional` may remain internal/hidden while preserving semantic contract.

## 7. Required

Fail closed:

- machine package/job overlay required and not current => block;
- exact program-bound proof absent => block;
- Pending/Unknown/Stale/Collision => block;
- environment/setup/calibration/tool/capability/interpolation identity mismatch => block;
- cannot automatically downgrade to Optional/Disabled.

## 8. Certificate Identity

Required-mode certificate key eventually covers:

```text
planHash
blockHash
interpolationModelHash/version
machine package/environment revision
setup/calibration/tool envelope revision
controller capability/dynamics semantic hash
collision policy/tolerance revision
```

Current endpoint/node certificate identity is not sufficient for optimized primitives whose continuous interpolation changes.

## 9. Scheduling

```text
FinalMotionPlan published
      |
      +-- Disabled -> Ready (non-collision gates only), no collision job
      |
      +-- Optional -> Ready + optional async proof attachment
      |
      +-- Required -> Certifying -> ReadyWithCurrentProof -> executable
```

Proof publication can attach only to the same immutable plan identity and never changes blocks.

## 10. Tests

1. valid kinematics + no machine model + Disabled -> planning succeeds;
2. valid FinalMotionPlan + no collision resources + Disabled -> Process preflight passes non-collision gate;
3. same setup + Required -> fails closed;
4. Disabled path backend constructor/query count = 0;
5. current hard `collision.complete/Pending` checks are proven policy-scoped;
6. initial approach does not accidentally retain an unconditional collision-certificate requirement in Disabled;
7. switching controller mode or collision policy invalidates the prepared run as specified;
8. Required old proof cannot attach after plan/interpolation/context change.
