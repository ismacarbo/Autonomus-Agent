# Local planner patch

The validated hardware build includes a local change to
`third_party/progettotesi/src/lib/behaviours.cpp`: the upstream gate-pass distance
is too large for the compact robot. This patch preserves that existing change
without requiring write access to the upstream planner repository.

After cloning or updating this repository on either workstation or Raspberry Pi:

```sh
git submodule update --init --recursive
bash scripts/apply_planner_patch.sh
```

Run these commands before configuring/building. The script checks whether the
patch is already present and leaves incompatible local changes untouched rather
than overwriting them. A modified `third_party/progettotesi` working tree is
expected after application; the parent repository still pins the original
submodule commit. Do not commit a new submodule pointer unless that commit is
also available from a configured remote.

`progettotesi-small-robot-gate-distance.patch` was exported from the local change
already used in the October 8, 2026 validation, not introduced as a new tuning.
