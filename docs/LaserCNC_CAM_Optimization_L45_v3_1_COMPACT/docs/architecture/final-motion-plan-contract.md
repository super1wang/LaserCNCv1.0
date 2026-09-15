# FinalMotionPlan 公共合同 — FINAL

## 1. Purpose

`FinalMotionPlan` 是 CAM 发布给 Process / Simulation / optional Collision Certifier 的唯一执行运动真相。它描述**已经完成完整运动学求解、Full5D 优化、候选筛选并选择完成**的运动，不是原始 sampled toolpath，也不是控制器 SDK command list。

优先扩展仓库现有 `CamMotionPlanSnapshot`，不得为了本规划建立第二套独立 execution plan。

## 2. Orthogonal Dimensions

```cpp
enum class MotionClass : uint8_t {
    SingleAxis,
    Coordinated2D,
    Coordinated3D,
    Reduced4D,
    Full5D
};

enum class ControllerMotionMode : uint8_t {
    PhysicalAxes,
    RTCP
};

enum class CollisionVerificationMode : uint8_t {
    Disabled,
    Optional,
    Required
};
```

`MotionClass` MUST NOT contain `RTCP` in its canonical name. RTCP is command-coordinate/controller semantic policy, not geometric DOF classification.

## 3. Recommended Semantic Shape

Exact private field names MAY follow repository style; semantics MUST remain:

```cpp
struct MotionCompilationContext {
    uint64_t workspaceGeneration{};
    uint64_t sourceToolpathRevision{};
    QByteArray contourOrderHash;
    QByteArray machineKinematicsHash;
    QByteArray setupCalibrationHash;
    QByteArray toolProcessHash;
    QByteArray optimizationPolicyHash;
    ControllerMotionMode controllerMode{};
    QByteArray controllerCapabilityHash;
    QByteArray dynamicsSemanticHash;
    CollisionVerificationMode collisionMode{};
    uint32_t interpolationModelVersion{};
};

struct CamMotionBlock {
    uint64_t blockId{};
    CamMotionPhase phase{};
    uint64_t contourId{};
    MotionClass motionClass{MotionClass::Full5D};
    OptimizationState optimizationState{OptimizationState::Raw};
    MotionInterpolationKind interpolation{};
    AxisMask activeAxisMask{};

    // controller-independent semantic data
    QVector<PhysicalAxisPose> physicalKnots; // machine physical-layout order
    QVector<ReferenceTcpPose> referenceTcpKnots; // when meaningful
    SourceSpanSet sourceSpans;
    ProcessFenceSet fences;
    MotionFeedSemantics feed;
    MotionToleranceProof toleranceProof;

    QByteArray blockHash;
};

struct CamFinalMotionPlanSnapshot {
    uint64_t revision{};
    MotionCompilationContext context;
    QVector<CamMotionBlock> blocks;
    QByteArray planHash;
    QString solverId;
    int solverVersion{};
    QString failureReason;
};
```

The shown types are semantic sketches, not mandatory private C++ spellings.

## 4. Identity

`planHash/contextHash` must change when execution semantics may change, including:

- source toolpath/order；
- machine kinematics/layout/limits；
- setup/WPC/calibration；
- tool/process recipe if it affects motion selection/feed semantics；
- path-affecting optimizer tolerance/policy；
- selected `ControllerMotionMode` when compatibility/command semantics change；
- controller capability/qualification revision used in candidate admissibility；
- interpolation model/version；
- block source spans/active mask/knots/process fences。

Collision proof identity is separate attachment identity but references the exact plan/block hashes.

## 5. Controller Compatibility

Candidate selection MUST happen before publication.

A published block may be lowered only if its frozen compatibility contract is satisfied by the selected `ControllerMotionMode` and capability revision.

If the operator toggles RTCP and that changes representability:

```text
old plan != silently re-encoded alternate path

required action:
CAM candidate re-admission / re-selection
 -> new FinalMotionPlan identity
 -> new PreparedDeviceProgram
```

Process/lowering MUST NOT select “Optimized Full5D fallback” after publication. That fallback/candidate choice belongs in CAM before publication.

## 6. Legacy Projection

During migration:

- `pointsByContourId` and legacy `nodes` may remain for display/compatibility;
- they must be derived from the exact plan or explicitly tagged legacy-only;
- new execution schema cannot consume them as an independent motion truth;
- a derived projection should record `derivedFromPlanHash` when practical;
- reverse mutation from legacy points back into FinalMotionPlan is forbidden.

## 7. Process Consumer Contract

Process:

- validates plan identity/context compatibility once during preparation;
- builds `PreparedDeviceProgram` with frozen tool/IO/profile/mode;
- executes blocks in published order;
- does not re-IK, resample for optimization, reduce/restore axes, reorder, or change fences;
- may perform an explicitly specified controller-equivalent lowering/tessellation only when the contract proves equivalence;
- applies collision proof gate only according to frozen `CollisionVerificationMode`.

## 8. Collision Binding

For Required collision qualification, proof identity must cover at least:

```text
program/plan hash
block hash
interpolation model hash/version
machine package/environment revision
setup/calibration/tool envelope revision
controller capability/dynamics semantic revision
collision policy/tolerance revision
```

A proof for endpoints/nodes alone is insufficient to certify a different continuous interpolation.

## 9. Acceptance

- MotionClass and ControllerMotionMode are separately represented/tested;
- Full5D + PhysicalAxes and Full5D + RTCP are both representable without separate CAM planners;
- reduced candidates are published only when selected controller mode can represent them exactly;
- Process cannot switch candidate after publication;
- legacy point arrays cannot become a second execution truth;
- Disabled collision mode does not require proof fields to be complete;
- deterministic repeated compilation of same context gives same planHash/selection, excluding explicitly nonsemantic metadata.
