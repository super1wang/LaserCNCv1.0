# GTN Controller Lowering — PhysicalAxes / RTCP 双编码合同

## 1. Responsibility

GTN lowering converts an immutable `FinalMotionPlan` into GTN commands. It is an encoder/adaptor, not a planner.

```text
FinalMotionPlan
       |
       +-- ControllerMotionMode::PhysicalAxes
       |       -> explicit physical-layout axis mapping -> GTN ACS semantics
       |
       +-- ControllerMotionMode::RTCP
               -> tcpMcs + rotary orientation -> GTN Group RTCP semantics
```

Lowering MUST NOT redo IK, choose another MotionClass, reduce/restore DOF, reorder blocks, change source geometry, or silently fall back to raw points.

## 2. Repository Semantics To Preserve

Current system already distinguishes RTCP and non-RTCP Group coordinate semantics. Existing `ToolpathExportPoint::tcpMcs` is a table-zero reference TCP with carrier motion removed, not world TCP and not a generic physical-axis coordinate.

The final implementation must preserve existing fault/release/profile synchronization and device queue authority rather than building a parallel execution stack.

## 3. ControllerMotionMode::RTCP

Baseline Full5D lowering:

```text
command target = { tcpMcsX, tcpMcsY, tcpMcsZ, rotary-1, rotary-2 }
Group command coordinate = existing RTCP/MCS semantic
orientation = existing rotary-axis-pos semantic
```

Requirements:

- use the `tcpMcs` belonging to the exact selected block/knots;
- keep established unwrapped rotary turn intent;
- continue endpoint transform-agreement validation where currently applicable;
- whole-segment equivalence for optimized/merged/native blocks is a separate semantic qualification using `ContinuousMotionEvaluator`;
- endpoint equality alone does not prove whole-segment equality.

## 4. ControllerMotionMode::PhysicalAxes

Baseline lowering:

```text
FinalMotionPlan physical-layout axis pose
  -> validate explicit MachineAxisLayout mapping
  -> map physical roles/indices to controller axes
  -> GTN non-RTCP / ACS command semantics
```

Requirements:

- use CAM's solved physical pose; no inverse/forward re-solve in Process;
- do not reinterpret `RapidPose.axes` semantic `[X,Y,Z,R1,R2]` as arbitrary physical layout;
- mapping must be explicit and fingerprinted;
- all target axes finite/in-limit before command preparation;
- feed semantics must be qualified for the selected GTN mode.

## 5. Compatibility Matrix

| MotionClass | PhysicalAxes | RTCP | Default release state |
|---|---|---|---|
| Full5D | exact physical axis mapping | `tcpMcs + rotary` | baseline path after software qualification |
| Reduced4D | only with exact inactive-axis hold semantics | only with exact RTCP-equivalent physical semantics | candidate-gated |
| Coordinated3D/2D | same | same | candidate-gated |
| SingleAxis | direct/group semantics must preserve ownership/process timing | only if RTCP representation yields the same intended physical motion | candidate-gated |
| native arc/cylinder | capability + HIL qualified | capability + HIL qualified | disabled until qualified |

Unsupported cell => candidate rejected before FinalMotionPlan publication. It is NOT a lowering fallback decision.

## 6. Capability State

For special/native behaviors use qualification states:

```text
Unknown
Documented
SDKVerified
HILQualified
Rejected
```

Only states permitted by release policy may be selected automatically. For new native real-machine interpolation, default requirement is `HILQualified`.

Capability snapshot/hash must cover any semantic assumption used during candidate selection, including:

- group coordinate/profile/orientation mode;
- reduced-axis/group ownership behavior;
- command list IO ordering;
- native arc/cylinder support;
- feed/reference-axis interpretation;
- interpolation/blending semantics;
- firmware/SDK revision where relevant.

## 7. Feed / Dynamics

Do not copy one scalar mm/s into mixed linear/rotary motion without a defined path metric.

For cylinder surface parameterization with axial coordinate `u`, radius `R`, angle `theta` in radians:

```text
v_surface = sqrt((du/dt)^2 + (R*dtheta/dt)^2)
```

Pure C equivalent:

```text
omega_deg_s = 180 * v_surface / (pi * R)
```

For Full5D, record and qualify the GTN Group command-velocity reference axes/ratios and effective profile limits. If a mixed PhysicalAxes feed cannot be mapped conservatively, that controller-mode/candidate pair is inadmissible rather than guessed.

## 8. PreparedDeviceProgram

Lowering output is a frozen program including:

- planHash/contextHash;
- ControllerMotionMode;
- capability/qualification revision;
- blockId/knot/native-command mapping;
- frozen effective tool/process/IO settings;
- effective motion/profile semantic values;
- runEpoch;
- encoding hash.

The current run must not re-read mutable tool/UI values after preparation to alter already selected motion semantics.

## 9. Device Queue / Group Session

- all GTN calls use existing `DeviceCommandQueue` / existing device authority;
- a future `GroupSession` may consolidate lifecycle state, not become a second scheduler;
- direct point/axis motion requiring axis ownership must not run while Group owns the axes;
- preserve stop/fault/release and profile synchronization protections;
- filling large lists may be sliced through queue scheduling for Stop responsiveness; this does not imply a vendor call is preemptible;
- unknown Start outcome => outputs safe/off + stop/latch + no replay.

## 10. Invariants

- **GTN-INV-01** lowering is pure semantic encoding, not planning.
- **GTN-INV-02** RTCP and PhysicalAxes use the same selected FinalMotionPlan identity.
- **GTN-INV-03** unsupported lowering combination is rejected before publish or blocks on identity mismatch; no alternate trajectory is picked in Process.
- **GTN-INV-04** `tcpMcs` is used only under its documented reference-TCP semantics.
- **GTN-INV-05** physical axis mapping is explicit and fingerprinted.
- **GTN-INV-06** activeAxisMask != Group membership.
- **GTN-INV-07** native/special mode is qualification-gated.
- **GTN-INV-08** IO order/fences survive lowering exactly.
- **GTN-INV-09** all vendor calls remain under existing command authority.
