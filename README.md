# fw-proto-i2c — I2C protocol kit

A Featherweight-HDL **protocol kit** that bridges class-level
controller/target/monitor APIs to the signal-level I2C bus (NXP/Philips
UM10204), built on the `fw-proto-kit` pattern (six elements per role).
Vendored under `fwvip-i2c/packages/` the same way `fw-proto-chi` and
`fw-proto-wb` are vendored under their respective VIP repos.

**Status:** controller, target and monitor roles implemented for Standard/
Fast/Fast-plus timing with 7-bit addressing (10-bit, Hs-mode and UFm are
deferred -- design.md §8). Both required back-to-back sims pass, as does
the first at 1-clock timing (the fastest bus the cores follow). The
back-to-back BMC passes at depth 80 with every cover goal reached; see
[`tests/formal/PROPERTIES.md`](tests/formal/PROPERTIES.md). See
[`../../docs/design.md`](../../docs/design.md) in the parent `fwvip-i2c`
checkout for the full design.

## Layout

```
src/            i2c_{controller,target,monitor}_if.svh / _bridge.svh /
                _xtor_if.sv / _xtor_core.sv / _xtor.sv
tests/          i2c_proto_tb.sv          (required back-to-back sim test)
                i2c_proto_fast_tb.sv     (the same at 1-clock timing)
tests/arb/      i2c_arb_tb.sv            (required 2-controller arbitration test)
tests/formal/   i2c_proto_fv.sv + PROPERTIES.md  (required formal component)
tests/lint/     i2c_lint_top.sv          (lint elaboration top)
```

## Build & run

`flow.yaml` builds on the `project.dv` archetype (dv-flow-libproject), so
the standard verbs apply:

```bash
export IVPM_PACKAGES=<...>/fwvip-i2c/packages
export PATH=$IVPM_PACKAGES/python/bin:$IVPM_PACKAGES/verilator/bin:$IVPM_PACKAGES/yosys/bin:$PATH

dfm run tests        # the back-to-back sims (i2c_proto, i2c_proto_fast, i2c_arb)
dfm run smoke        # what CI runs on every commit
dfm run tests-info   # the case inventory, building nothing
dfm run lint-rtl     # Verilator lint of the cores, held to zero findings
dfm run fv           # back-to-back formal BMC
dfm run fv-cover     # formal cover goals (non-vacuity)
```
