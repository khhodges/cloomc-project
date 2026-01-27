// ============================================================================
// CTMM CALL Church-Instruction (CLOOMC)
// ============================================================================
// This module implements the CALL instruction which loads a capability
// from a C-List into CR7 (Nucleus) for invoking kernel services.
//
// Syntax: CALL CRs[Index]
//   CRs = Source C-List register (CR0-CR6)
//   Index = Index into the C-List
//   Destination = CR7 (Nucleus) - hardwired
//
// CALL Steps:
//   1. Verify source CRs is in range 0-6 (not reserved CR7/CR8/CR15)
//   2. Verify source CRs has L (Load) permission
//   3. Call mLoad with sub_cr_dst = CR7 (hardwired destination)
//
// The actual capability fetching is done by ctmm_mload.sv
// This reduces the Trusted Computing Base - all Church CLOOMC instructions
// share the same verified mLoad micro-routine for capability fetching.
//
// FAULT conditions:
//   - Source CRs not in range 0-6
//   - Source CRs lacks L permission
//   - mLoad faults (bounds, MAC, etc.)
// ============================================================================

module ctmm_call
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        call_start,           // Start CALL execution
    input  logic [3:0]  cr_src,               // Source register (CRs) - must be CR0-CR6
    input  logic [7:0]  index,                // C-List index
    output logic        call_busy,            // CALL in progress
    output logic        call_complete,        // CALL finished successfully
    output logic        call_fault,           // CALL caused a fault
    output fault_type_t fault_type,           // Type of fault
    
    // Capability register read interface (directly from subroutine)
    output logic [3:0]  cr_rd_addr,           // Register to read
    input  capability_reg_t cr_rd_data,       // Full 256-bit register data
    
    // Capability register write interface (directly from subroutine)
    output logic [3:0]  cr_wr_addr,           // Register to write (always CR7)
    output capability_reg_t cr_wr_data,       // Full 256-bit data to write
    output logic        cr_wr_en,             // Write enable
    
    // CR15 (Namespace) interface
    input  capability_reg_t cr15_namespace,   // CR15 Namespace register
    
    // Memory interface
    output logic [63:0] mem_addr,             // Memory address
    output logic        mem_rd_en,            // Read enable
    input  logic [63:0] mem_rd_data,          // Read data
    input  logic        mem_rd_valid,         // Read data valid
    
    // Thread update interface - writes GT (G=0) to Thread[CR7]
    output logic        thread_wr_en,         // Write enable for Thread[CR7]
    output logic [3:0]  thread_wr_idx,        // Index into Thread (= CR7 = 4'd7)
    output logic [63:0] thread_wr_data,       // GT with G=0
    
    // G bit reset interface
    output logic        g_bit_reset,
    output logic [63:0] g_bit_addr
);

    // ========================================================================
    // Constants
    // ========================================================================
    
    localparam logic [3:0] CR7_NUCLEUS = 4'd7;     // Hardwired destination
    localparam logic [3:0] MAX_CLIST_REG = 4'd6;   // Maximum allowed source register
    
    // ========================================================================
    // State Machine - CALL instruction wrapper
    // ========================================================================
    
    typedef enum logic [2:0] {
        CALL_IDLE,
        CALL_CHECK_SRC,       // Verify source is CR0-CR6
        CALL_READ_SRC,        // Read source register for permission check
        CALL_CHECK_PERM,      // Verify L permission on source
        CALL_CALL_SUB         // Call mLoad and wait for completion
    } call_state_t;
    
    call_state_t state, next_state;
    
    // ========================================================================
    // Local Register Read Control
    // ========================================================================
    
    logic        local_cr_rd_en;
    logic [3:0]  local_cr_rd_addr;
    logic [3:0]  sub_cr_rd_addr;
    
    assign local_cr_rd_en = (state == CALL_CHECK_SRC) || (state == CALL_READ_SRC);
    assign local_cr_rd_addr = cr_src;
    
    // ========================================================================
    // Permission Check Logic
    // ========================================================================
    
    logic src_in_range;
    logic src_has_l_perm;
    logic [9:0] src_perms;
    
    capability_reg_t src_reg_latched;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_reg_latched <= '0;
        end else if (state == CALL_READ_SRC) begin
            src_reg_latched <= cr_rd_data;
        end
    end
    
    assign src_in_range = (cr_src <= MAX_CLIST_REG);
    assign src_perms = src_reg_latched.word0_gt[57:48];
    assign src_has_l_perm = src_perms[PERM_L];
    
    // ========================================================================
    // Fault Latching
    // ========================================================================
    
    logic        fault_latched;
    fault_type_t fault_type_latched;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_latched <= 1'b0;
            fault_type_latched <= FAULT_NONE;
        end else if (state == CALL_IDLE) begin
            fault_latched <= 1'b0;
            fault_type_latched <= FAULT_NONE;
        end else if (state == CALL_CHECK_SRC && !src_in_range) begin
            fault_latched <= 1'b1;
            fault_type_latched <= FAULT_PERM;
        end else if (state == CALL_CHECK_PERM && !src_has_l_perm) begin
            fault_latched <= 1'b1;
            fault_type_latched <= FAULT_PERM;
        end else if (state == CALL_CALL_SUB && sub_fault_latched) begin
            fault_latched <= 1'b1;
            fault_type_latched <= sub_fault_type;
        end
    end
    
    // ========================================================================
    // mLoad Subroutine Instance
    // ========================================================================
    
    logic        sub_start;
    logic        sub_start_reg;
    logic        sub_busy;
    logic        sub_done;
    logic        sub_fault;
    logic        sub_done_latched;
    logic        sub_fault_latched;
    fault_type_t sub_fault_type;
    
    // Single-cycle pulse on entry to CALL_CALL_SUB
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sub_start_reg <= 1'b0;
        else if (state == CALL_CHECK_PERM && next_state == CALL_CALL_SUB)
            sub_start_reg <= 1'b1;
        else
            sub_start_reg <= 1'b0;
    end
    
    // Sticky latches for completion/fault
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_done_latched <= 1'b0;
            sub_fault_latched <= 1'b0;
        end else if (state == CALL_IDLE) begin
            sub_done_latched <= 1'b0;
            sub_fault_latched <= 1'b0;
        end else begin
            if (sub_done) sub_done_latched <= 1'b1;
            if (sub_fault) sub_fault_latched <= 1'b1;
        end
    end
    
    assign sub_start = sub_start_reg;
    
    ctmm_mload u_mload (
        .clk            (clk),
        .rst_n          (rst_n),
        
        // Subroutine interface
        .sub_start      (sub_start),
        .sub_cr_src     (cr_src),
        .sub_cr_dst     (CR7_NUCLEUS),    // Hardwired to CR7
        .sub_index      (index),
        .sub_busy       (sub_busy),
        .sub_done       (sub_done),
        .sub_fault      (sub_fault),
        .sub_fault_type (sub_fault_type),
        
        // Register interfaces
        .cr_rd_addr     (sub_cr_rd_addr),
        .cr_rd_data     (cr_rd_data),
        .cr_wr_addr     (cr_wr_addr),
        .cr_wr_data     (cr_wr_data),
        .cr_wr_en       (cr_wr_en),
        
        // CR15 interface
        .cr15_namespace (cr15_namespace),
        
        // Memory interface
        .mem_addr       (mem_addr),
        .mem_rd_en      (mem_rd_en),
        .mem_rd_data    (mem_rd_data),
        .mem_rd_valid   (mem_rd_valid),
        
        // Thread update interface
        .thread_wr_en   (thread_wr_en),
        .thread_wr_idx  (thread_wr_idx),
        .thread_wr_data (thread_wr_data),
        
        // G bit reset interface
        .g_bit_reset    (g_bit_reset),
        .g_bit_addr     (g_bit_addr)
    );
    
    // ========================================================================
    // State Register
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= CALL_IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    // ========================================================================
    // Next State Logic
    // ========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            CALL_IDLE: begin
                if (call_start)
                    next_state = CALL_CHECK_SRC;
            end
            
            CALL_CHECK_SRC: begin
                if (!src_in_range)
                    next_state = CALL_IDLE;  // Fault
                else
                    next_state = CALL_READ_SRC;
            end
            
            CALL_READ_SRC: begin
                // Wait for register data to be valid
                next_state = CALL_CHECK_PERM;
            end
            
            CALL_CHECK_PERM: begin
                if (!src_has_l_perm)
                    next_state = CALL_IDLE;  // Fault
                else
                    next_state = CALL_CALL_SUB;
            end
            
            CALL_CALL_SUB: begin
                // Wait for mLoad to complete
                if (sub_done_latched || sub_fault_latched)
                    next_state = CALL_IDLE;
            end
            
            default: next_state = CALL_IDLE;
        endcase
    end
    
    // ========================================================================
    // Register Read Address Muxing
    // ========================================================================
    
    assign cr_rd_addr = local_cr_rd_en ? local_cr_rd_addr : sub_cr_rd_addr;
    
    // ========================================================================
    // Output Signals
    // ========================================================================
    
    assign call_busy = (state != CALL_IDLE);
    assign call_complete = (state == CALL_CALL_SUB) && sub_done_latched;
    assign call_fault = fault_latched;
    assign fault_type = fault_type_latched;

endmodule
