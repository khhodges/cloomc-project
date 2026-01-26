// ============================================================================
// CTMM Permission Checker - Hardware Permission Validation
// ============================================================================
// Implements capability-based access control via Golden Token permissions
// All security checks are performed here - single FAULT output for failsafe
// ============================================================================

module ctmm_perm_check
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Permission check request
    input  golden_token_t gt_in,          // Golden Token to check
    input  logic [15:0] required_perms,   // Required permission mask
    input  logic        check_valid,      // Check request valid
    
    // Bounds checking
    input  logic [31:0] access_index,     // Index being accessed
    input  logic [63:0] limit,            // Limit from namespace entry
    input  logic        check_bounds,     // Enable bounds check
    
    // MAC validation
    input  logic [63:0] calculated_mac,   // MAC calculated from data
    input  logic [63:0] stored_mac,       // MAC from namespace entry
    input  logic        check_mac,        // Enable MAC check
    
    // Results
    output logic        perm_granted,     // All permissions granted
    output logic        bounds_ok,        // Bounds check passed
    output logic        mac_valid,        // MAC validation passed
    output logic        all_checks_pass,  // All enabled checks pass
    output fault_type_t fault_type,       // Type of fault if any
    output logic        fault_valid,      // Fault occurred
    
    // G bit garbage collection output
    output logic        g_bit_set,        // G bit is set on this GT
    output logic        is_namespace_access // This is a namespace entry access
);

    // ========================================================================
    // Permission Check Logic
    // ========================================================================
    
    logic [15:0] gt_perms;
    assign gt_perms = gt_in.perms;
    
    // Check if GT is null (offset and perms both zero)
    logic is_null_gt;
    assign is_null_gt = (gt_in.offset == 32'h0) && (gt_in.perms == 16'h0);
    
    // Check if all required permissions are present
    logic perms_match;
    assign perms_match = ((gt_perms & required_perms) == required_perms);
    
    // Permission granted if not null and permissions match
    assign perm_granted = !is_null_gt && perms_match;
    
    // ========================================================================
    // Bounds Check Logic
    // ========================================================================
    
    // Access index must be less than limit
    assign bounds_ok = !check_bounds || (access_index < limit[31:0]);
    
    // ========================================================================
    // MAC Validation Logic
    // ========================================================================
    
    // MAC must match exactly
    assign mac_valid = !check_mac || (calculated_mac == stored_mac);
    
    // ========================================================================
    // G Bit and Namespace Access Detection
    // ========================================================================
    
    // G bit is set if permission bit 9 is high
    assign g_bit_set = gt_perms[PERM_G];
    
    // Namespace access if M permission is set (hardware-level access)
    assign is_namespace_access = gt_perms[PERM_M] || gt_perms[PERM_L];
    
    // ========================================================================
    // Combined Result
    // ========================================================================
    
    assign all_checks_pass = perm_granted && bounds_ok && mac_valid;
    
    // ========================================================================
    // Fault Detection - Single FAULT for failsafe security
    // ========================================================================
    
    always_comb begin
        fault_valid = 1'b0;
        fault_type = FAULT_NONE;
        
        if (check_valid) begin
            if (is_null_gt) begin
                fault_valid = 1'b1;
                fault_type = FAULT_NULL_CAP;
            end else if (!perms_match) begin
                fault_valid = 1'b1;
                // Determine which permission failed (first one found)
                if ((required_perms & PERM_MASK_R) && !(gt_perms & PERM_MASK_R))
                    fault_type = FAULT_PERM_R;
                else if ((required_perms & PERM_MASK_W) && !(gt_perms & PERM_MASK_W))
                    fault_type = FAULT_PERM_W;
                else if ((required_perms & PERM_MASK_X) && !(gt_perms & PERM_MASK_X))
                    fault_type = FAULT_PERM_X;
                else if ((required_perms & PERM_MASK_L) && !(gt_perms & PERM_MASK_L))
                    fault_type = FAULT_PERM_L;
                else if ((required_perms & PERM_MASK_S) && !(gt_perms & PERM_MASK_S))
                    fault_type = FAULT_PERM_S;
                else if ((required_perms & PERM_MASK_E) && !(gt_perms & PERM_MASK_E))
                    fault_type = FAULT_PERM_E;
                else if ((required_perms & PERM_MASK_M) && !(gt_perms & PERM_MASK_M))
                    fault_type = FAULT_PERM_M;
                else
                    fault_type = FAULT_PERM_R; // Default
            end else if (check_bounds && !bounds_ok) begin
                fault_valid = 1'b1;
                fault_type = FAULT_BOUNDS;
            end else if (check_mac && !mac_valid) begin
                fault_valid = 1'b1;
                fault_type = FAULT_MAC;
            end
        end
    end

endmodule
