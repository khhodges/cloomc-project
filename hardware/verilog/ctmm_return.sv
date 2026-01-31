// ============================================================================
// CTMM RETURN Instruction
// ============================================================================
// Implements the RETURN instruction which returns from a procedure call
// by restoring the saved context and jumping to the return address.
//
// Syntax: RETURN CRn
//   CRn = Register containing return capability (saved by CALL)
//
// RETURN Steps:
//   1. Read return capability from CRn
//   2. Verify CRn has E (Enter) permission
//   3. Restore CR6 (C-List) from return capability
//   4. Restore CR7 (Nucleus) from return capability  
//   5. Set NIA to saved return address
//   6. Clear internal abstraction state
//
// The return capability structure (saved by CALL):
//   Word 0: GT with saved permissions
//   Word 1: Return NIA (instruction address)
//   Word 2: Saved CR6 offset
//   Word 3: Saved CR7 offset
//
// FAULT conditions:
//   - CRn lacks E permission
//   - CRn is null capability
// ============================================================================

module ctmm_return
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        return_start,         // Start RETURN execution
    input  logic [2:0]  cr_src,               // Source register (3-bit: CR0-CR7)
    output logic        busy,                 // RETURN in progress
    output logic        complete,             // RETURN finished
    output logic        fault_valid,          // RETURN caused a fault
    output fault_type_t fault_type,           // Type of fault
    
    // Capability register read interface
    output logic [3:0]  cr_rd_addr,           // Register to read
    input  capability_reg_t cr_rd_data,       // Full 256-bit register data
    
    // Capability register write interface (for CR6, CR7)
    output logic [3:0]  cr_wr_addr,           // Register to write
    output capability_reg_t cr_wr_data,       // Data to write
    output logic        cr_wr_en,             // Write enable
    
    // NIA update interface
    output logic        nia_set,              // Set NIA enable
    output logic [63:0] nia_value,            // New NIA value
    
    // M bit clear interface (leaving internal abstraction)
    output logic        clear_m_bit           // Clear M bit on current context
);

    // ========================================================================
    // Constants
    // ========================================================================
    
    localparam logic [3:0] CR6_CLIST = 4'd6;
    localparam logic [3:0] CR7_NUCLEUS = 4'd7;
    
    // ========================================================================
    // State Machine
    // ========================================================================
    
    typedef enum logic [2:0] {
        IDLE,
        READ_SRC,             // Read source register (return capability)
        CHECK_PERM,           // Check E permission
        RESTORE_CR6,          // Restore CR6 (C-List)
        RESTORE_CR7,          // Restore CR7 (Nucleus)
        SET_NIA,              // Set NIA to return address
        COMPLETE,
        FAULT
    } state_t;
    
    state_t state, next_state;
    
    // ========================================================================
    // Latched Data
    // ========================================================================
    
    capability_reg_t return_cap;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            return_cap <= '0;
        else if (state == READ_SRC)
            return_cap <= cr_rd_data;
    end
    
    // ========================================================================
    // Permission Check
    // ========================================================================
    
    logic has_e_perm;
    logic is_null_cap;
    logic [9:0] src_perms;
    
    assign src_perms = return_cap.word0_gt.perms[9:0];
    assign has_e_perm = src_perms[PERM_E];
    assign is_null_cap = (return_cap.word0_gt == GT_NULL);
    
    // ========================================================================
    // Extracted Return Values
    // ========================================================================
    // Return capability structure (saved by CALL):
    //   Word 0: GT with E permission (identifies return point)
    //   Word 1: Return NIA (instruction address to resume)
    //   Word 2: Saved CR6.GT (C-List Golden Token)
    //   Word 3: Saved CR7.GT (Nucleus Golden Token)
    //
    // To fully restore CR6/CR7, we need to re-fetch from namespace
    // using the saved GT offsets. This is done via mLoad interface.
    // ========================================================================
    
    logic [63:0] saved_nia;
    golden_token_t saved_cr6_gt;
    golden_token_t saved_cr7_gt;
    
    assign saved_nia = return_cap.word1_location;
    assign saved_cr6_gt = return_cap.word2_limit;  // Full GT for CR6
    assign saved_cr7_gt = return_cap.word3_seals;  // Full GT for CR7
    
    // ========================================================================
    // State Register
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end
    
    // ========================================================================
    // Fault Latching
    // ========================================================================
    
    fault_type_t fault_latched;
    logic fault_flag;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_flag <= 1'b0;
            fault_latched <= FAULT_NONE;
        end else if (state == IDLE) begin
            fault_flag <= 1'b0;
            fault_latched <= FAULT_NONE;
        end else if (state == CHECK_PERM) begin
            if (is_null_cap) begin
                fault_flag <= 1'b1;
                fault_latched <= FAULT_NULL_CAP;
            end else if (!has_e_perm) begin
                fault_flag <= 1'b1;
                fault_latched <= FAULT_PERM_E;
            end
        end
    end
    
    // ========================================================================
    // Next State Logic
    // ========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (return_start)
                    next_state = READ_SRC;
            end
            
            READ_SRC: next_state = CHECK_PERM;
            
            CHECK_PERM: begin
                if (is_null_cap || !has_e_perm)
                    next_state = FAULT;
                else
                    next_state = RESTORE_CR6;
            end
            
            RESTORE_CR6: next_state = RESTORE_CR7;
            RESTORE_CR7: next_state = SET_NIA;
            SET_NIA: next_state = COMPLETE;
            
            COMPLETE: next_state = IDLE;
            FAULT: next_state = IDLE;
            
            default: next_state = IDLE;
        endcase
    end
    
    // ========================================================================
    // Output Signals
    // ========================================================================
    
    assign busy = (state != IDLE);
    assign complete = (state == COMPLETE);
    assign fault_valid = fault_flag;
    assign fault_type = fault_latched;
    
    // CR read (source register) - 3-bit CR expanded to 4-bit address
    assign cr_rd_addr = {1'b0, cr_src};
    
    // CR write (restore CR6 or CR7 with full GT)
    // Note: Full capability restoration requires namespace fetch via mLoad
    // For now, we restore the GT directly - the caller should use LOAD
    // to fully populate Word1-3 from namespace if needed.
    always_comb begin
        cr_wr_addr = 4'h0;
        cr_wr_data = '0;
        cr_wr_en = 1'b0;
        
        case (state)
            RESTORE_CR6: begin
                cr_wr_addr = CR6_CLIST;
                cr_wr_data.word0_gt = saved_cr6_gt;
                // Word1-3 will be populated by subsequent namespace access
                // when the capability is actually used
                cr_wr_en = 1'b1;
            end
            
            RESTORE_CR7: begin
                cr_wr_addr = CR7_NUCLEUS;
                cr_wr_data.word0_gt = saved_cr7_gt;
                // Word1-3 will be populated by subsequent namespace access
                cr_wr_en = 1'b1;
            end
            
            default: ;
        endcase
    end
    
    // NIA update
    assign nia_set = (state == SET_NIA);
    assign nia_value = saved_nia;
    
    // Clear M bit when leaving abstraction
    assign clear_m_bit = (state == SET_NIA);

endmodule
