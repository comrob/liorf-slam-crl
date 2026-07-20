# PARAMETERS_RESTRUCTURE.md
This document describes a proposed restructuring of the parameter system in LIO-SAM to reduce unnecessary rebuilds and improve maintainability.

## Prompt
Every time I add a new parameter, I have to recompile the whole project because utility.h is included everywhere.
I want to keep the project easy to parse and debug for any new developer. At the same time, I do not want to rebuild the whole project every time I add a new parameter.
Please suggest low-risk simple stupid but effective and elegant refactoring to reach the desired behaviour.
Please do not do any implementations yet, just suggest the implementaiton paths for now.

-------

Solution suggested by Fable 5 goes below

## Why the full rebuild happens

Two separate problems are conflated in utility.h:

1. **Coupling**: `ParamServer` (member declarations + inline constructor) lives in a header included by every TU. Any new parameter touches that header → everything rebuilds.
2. **Cost per rebuild**: the same header drags in PCL (`registration/icp.h`, `range_image`), OpenCV, tf2, etc. into every TU, so each rebuild is also slow.

Fixing #1 kills the propagation; fixing #2 makes unavoidable rebuilds cheap. Suggested paths, ranked by value/risk:

## Path 1 (recommended core fix): per-subsystem parameter structs

- Create small plain-struct headers, e.g. `include/params/gps_params.hpp`, `params/loop_closure_params.hpp`, `params/frontend_params.hpp`, `params/backend_params.hpp` — you already do this for `DegeneracyDetectionParameters`, `PKOConfig`, `KdTreeLmConfig`, so it's an established pattern in the repo.
- Each struct gets a free loader function `declareAndLoad(rclcpp::Node&, GpsParams&)` **declared** in the struct's header, **defined** in one `src/params/*.cpp`.
- `ParamServer` holds the structs; nodes' `.cpp` files include only the param headers they actually use.
- Result: adding a GPS parameter touches `gps_params.hpp` + one `.cpp` + YAML → only GPS-consuming TUs rebuild (and loader logic lives in a `.cpp`, so tweaking defaults/loading rebuilds a single TU).
- Migration can be **incremental**: move one parameter group at a time; old public members can stay until their group is migrated. Low risk, always compiles.

Trade-off: call sites change from `gpsCovThreshold` to `params.gps.covThreshold` (mechanical rename, greppable, arguably more readable for new developers).

## Path 2 (zero-propagation convention for future params)

For parameters used by exactly one node/subsystem, stop routing them through `ParamServer` at all: declare and read them locally in the owning `.cpp` (e.g. inside `mapOptimization_gps.cpp`), near their use. New param = one `.cpp` + YAML, no header touched, nothing else rebuilds.

- Cheapest possible, works today with no refactor.
- Keep it disciplined: single-consumer params only, one clearly marked "parameters" block at the top of the file — otherwise it degrades into scattered config.

## Path 3 (complementary, big compile-time win): slim down utility.h

- Split into `utility.h` (stable helpers: enums, `publishCloud`, `ROS_TIME`, `QosPolicy`, IMU helpers) and `param_server.hpp` + `param_server.cpp` (constructor and `imuConverter` bodies out-of-line).
- Push heavy includes (`pcl/registration/icp.h`, `opencv2/opencv.hpp`, `pcl/range_image`) out of the header into the few `.cpp` files that actually need them.
- Doesn't stop propagation by itself, but often cuts per-TU compile time dramatically.

## Path 4 (no code change, tooling only)

- `ccache` and/or CMake `target_precompile_headers` for the stable PCL/GTSAM/rclcpp includes. Makes rebuilds cheaper regardless of structure. Near-zero risk.

## What I'd do

**Path 1 + Path 3** as the structural fix (incremental, matches your existing config-struct pattern, keeps everything explicit for new developers), with **Path 2** as the standing convention for single-node parameters, and Path 4 as free extra. 

Want me to start with one parameter group (e.g. GPS) as a proof-of-concept migration?