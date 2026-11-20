# Harvest Closure Implementation Plan

> **For agentic workers:** use this plan as the Stage 2 task checklist. Keep Stage 1 commits and tag available.

**Goal** Validate truth-driven harvest physics and then connect the unchanged real main controller to a synthetic vision service.

**Architecture** Stage 2A drives the existing `phase3` scene from its ground-truth cut point and measures contact, grasp, transport, dynamic release and settling. Stage 2B supplies the existing `/yolo_detection` contract from simulator geometry and runs the original single-stage main-controller pipeline.

**Tech Stack** ROS 2 Humble, Gazebo Classic, `gazebo_ros_state`, PyKDL, existing `xc_interfaces` services.

**Global constraints** Preserve the real robot workspace byte-for-byte. Keep the confirmed 180-degree arm mount and `machine_slide` rail. Run the work on `codex/harvest-closure-stage2`, based on Stage 1 commit `933efbd`.

---

## Stage 2A

- [x] Derive the lift sign from the expanded URDF transform chain. With the CAD datum's X-axis half-turn, local `-lift_height` raises the arm base in world Z.
- [x] Verify the v4 candidate station transform. At rail `1.25 m` and lift `0.50 m`, `arm_base_link` is at `[1.252, -0.004, 1.270379] m`. The cut point is `[1.199907, -0.713020, 2.075481] m`, or `[0.052093, 0.709020, 0.805102] m` in the arm base. Its IK and the service midpoint IK are reachable. At zero lift, target distance grows from `1.074063 m` to `1.486174 m`, and IK fails.
- [x] Independently sample `robot_carriage` from Gazebo `/get_entity_state` through three arm trajectories. World-X peak displacement was `0.000000040 mm`, while the `machine_slide` joint-state peak was `0.000000010 mm`. With an advancing clock received before timing started, the simulation advanced at `0.9996x` wall-clock. An earlier 80.47x estimate used an uninitialized clock origin and is invalid. `/clock` sampling used BEST_EFFORT QoS.
- [x] Cross-check the prior unbraked run against the Gazebo-reported physical wrist-link pose and reconstructed fixed-sphere target. The endpoint error is `9.836 mm` along the rail, corroborating the joint-sampled `10.379 mm` peak. The prior run did not preserve a physical carriage pose time series, so its exact peak cannot be reconstructed independently.
- [x] Add a point-transform regression test that fails against the current arbiter implementation.
- [x] Apply the point/vector frame corrections, verify the symmetric crossing-angle check, and test the blade contact points against real ODE sensor samples.
- [x] Set target peduncle collisions to `collide_without_contact` while preserving contact reporting. Verify blade contact, sub-0.4 mm finger closure and no hard-contact jitter.
- [x] Complete truth-driven `ATTACHED_TO_STEM -> HELD_BY_GRIPPER -> RELEASED`, including fruit transport and a velocity-based settled-in-basket check.

## Stage 2B

- [x] Confirm the real main controller defaults to `is_two_stage=False`. The wrist `/vision/tomato/get_pick_poses` service is only called from two-stage branches. Its eight-value unpacking mismatch is outside the default path, so the real workspace stays unmodified.
- [x] Implement a geometry-derived `/yolo_detection` service with aligned, same-order boxes and masks for `stem`, `tomato`, and `trunk`.
- [x] Run the unchanged main controller with a 1000-count-per-metre depth stream, matching the hard-coded scaling in its real-camera path. The actual motion service returned 30003 at midpoint inverse kinematics.
- [ ] Record skeleton pixel count, detected 3D point error to the scene cut point along axial and transverse axes, and three successful local-reset harvest cycles.

## Recovery Point

The Stage 1 recovery point is tag `geometry-actuators-2026-11-20` at commit `933efbd`.

## Measured completion state

Stage 2A passed two local-reset cycles at rail 1.25 m and lift 0.85 m. Fixes cover contact transforms, force-free target cutting, complete basket waypoints, unique-name dynamic release, fruit COM and inertia, and full 12-sphere settled containment. The first passing cycle converged after a 0.3 s endpoint lag. Continuous in-motion tracking error still needs measurement.

The geometry YOLO service and three-frame original-vision subchain passed. Each frame had 22 skeleton pixels and 3.9184 mm point error. The actual unchanged main process received detections and sent RPY [90, 70, 180] degrees. The motion adapter returned 30003 before moving to the midpoint. Full Stage 2B cycles remain blocked pending the actual wrist-to-cutter installation and SDK TCP reference.

Final unit verification passed 44 tests and the overlay built 9 packages. Detailed evidence is in `docs/stage2_harvest_acceptance.md` and `docs/evidence/stage2_harvest_2026-11-20.json`.
