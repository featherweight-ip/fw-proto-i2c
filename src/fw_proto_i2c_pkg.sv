// I2C protocol kit -- class layer. The transactor SV interfaces and modules
// (i2c_*_xtor*.sv) are separate compilation units (an interface/module
// cannot live in a package); they are listed alongside this file in the
// FileSet (src/flow.yaml).
//
// Deliberately fw-hdl-FREE and, like fw-proto-wb (the closest sibling kit),
// does NOT use the fw-hdl `fw_export`/`FW_*_IMP` macro convention the
// fw-proto-kit SKILL.md's ready/valid reference example uses -- the bridges
// here implement the interface-classes directly (plain `implements`), the
// same simpler shape fw_proto_wb_pkg.sv uses. This is a deviation from the
// SKILL.md's literal macro convention, following the proven sibling-kit
// precedent instead (see the kit implementation report for the full
// rationale).
`include "i2c_xtor_macros.svh"

package fw_proto_i2c_pkg;
    // API interface-classes.
    `include "i2c_controller_if.svh"
    `include "i2c_target_if.svh"
    `include "i2c_monitor_if.svh"

    // Transactor bridges -- hold a virtual transactor-interface and
    // implement (or drive) the API. They reference the transactor SV
    // interfaces by their (unmangled) names, so those interfaces must be
    // compiled in the same image.
    `include "i2c_controller_bridge.svh"
    `include "i2c_target_bridge.svh"
    `include "i2c_monitor_bridge.svh"

    // Transaction-level bus: controller API straight to a target model,
    // no transactors (the TLM views' I2C channel).
    `include "i2c_tlm_bus.svh"

endpackage
