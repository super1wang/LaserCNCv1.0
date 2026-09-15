# GTN Controller Qualification

## Stage 0 — Software/Mock

验证：

- Group RTCP optimized knots；
- userTag block mapping；
- IO sequence；
- endVelocityMode；
- fault/release；
- effective lookahead/smooth logs。

## Stage 1 — Controller Simulator / No Motion Hardware

若供应商 simulator 可用：

- long Full5D blocks；
- short/irregular baseline；
- multi-turn C；
- stop/restart；
- lookahead parameter sweep。

## Stage 2 — Real Machine, Laser Off

- 10% speed；
- 25%；
- 50%；
- nominal as permitted；
- compare raw vs optimized APOS/following error/vibration evidence。

## Stage 3 — Native Special Modes

Circle/Cylinder/Persistent Group 每项独立资格；未资格保持 feature flag off。

## Pass Rule

SDK 文档声称支持 ≠ 产品资格通过。只有 software implementation + simulator + required real-machine evidence 都满足，capability 才能标记 qualified。
