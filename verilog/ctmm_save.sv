// ============================================================================
// CTMM SAVE Church-Instruction (CLOOMC)
// ============================================================================
// This module implements the SAVE instruction which stores a Golden Token
// from a source CR into a C-List at a specified index.
//
// Syntax: SAVE CRs, CRd[Index]
//   CRs = Source CR (the capability whose GT we're saving)
//   CRd[Index] = Destination C-List slot
//
// SAVE Steps (optimized):
//   Step 1: Verify CRd in 0-6 AND initiate register read (parallel)
//   Step 2: Verify CRd has S (Save) permission
//   Step 3: Verify GT.B = TRUE - the Golden Token must allow being saved
//   Step 4: Verify Index < CRd.Limit
//   Step 5: Write CRs.Word0 (GT) to CRd.Location + Index*8
//
// Note: SAVE does not use mLoad - it's a simpler write-only operation.
// No MAC validation or G bit handling needed.
//
// FAULT conditions:
//   - Destination CRd not in range 0-6
//   - Destination CRd lacks S permission
//   - GT.B = FALSE (Golden Token cannot be saved)
//   - Index >= CRd.Limit (out of bounds)
// ============================================================================

module ctmm_save
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        save_start,           // Start SAVE execution
    input  logic [3:0]  cr_src,               // Source register (CRs) - GT to save
    input  logic [3:0]  cr_dst,               // Destination C-List (CRd) - must be CR0-CR6
    input  logic [7:0]  index,                // C-List index
    output logic        save_busy,            // SAVE in progress
    output logic        save_complete,        // SAVE finished successfully
    output logic        save_fault,           // SAVE caused a fault
    output fault_type_t fault_type,           // Type of fault
    
    // Capability register read interface
    output logic [3:0]  cr_rd_addr,           // Register to read
    input  capability_reg_t cr_rd_data,       // Full 256-bit register data
    
    // Memory write interface
    output logic [63:0] mem_wr_addr,          // Memory address to write
    output logic [63:0] mem_wr_data,          // Data to write (GT)
    output logic        mem_wr_en,            // Write enable
    input  logic        mem_wr_done           // Write complete acknowledgment
);

    // ========================================================================
    // Constants
    // ========================================================================
    
    localparam logic [3:0] MAX_CLIST_REG = 4'd6;  // Maximum allowed destination register
    
    // ========================================================================
    // State Machine
    // ========================================================================
    
    typedef enum logic [2:0] {
        SAVE_IDLE,
        SAVE_CHECK_DST_READ,  // Verify CRd in 0-6 AND initiate destination read
        SAVE_LATCH_DST,       // Wait for register data and latch destination
        SAVE_READ_SRC,        // Read source register to get GT
        SAVE_CHECK_S_BOUNDS,  // Verify S permission AND bounds (combined for speed)
        SAVE_WRITE_GT         // Write GT to memory
    } save_state_t;
    
    save_state_t state, next_state;
    
    // ========================================================================
    // Register Latches
    // ========================================================================
    
    capability_reg_t dst_reg_latched;  // Destination C-List register
    capability_reg_t src_reg_latched;  // Source register (for GT)
    
    // Latch destination data when it becomes valid (one cycle after read initiated)
    // Latch source data when it becomes valid (one cycle after read initiated)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dst_reg_latched <= '0;
            src_reg_latched <= '0;
        end else begin
            if (state == SAVE_LATCH_DST) begin
                dst_reg_latched <= cr_rd_data;  // Data valid after CHECK_DST_READ
            end
            if (state == SAVE_CHECK_S_BOUNDS) begin
                src_reg_latched <= cr_rd_data;  // Data valid after READ_SRC
            end
        end
    end
    
    // ========================================================================
    // Permission and Bounds Check Logic
    // ========================================================================
    
    logic dst_in_range;
    logic dst_has_s_perm;
    logic src_has_b_perm;
    logic index_in_bounds;
    logic [9:0] dst_perms;
    logic [9:0] src_perms;
    logic [63:0] dst_limit;
    logic [63:0] dst_location;
    
    assign dst_in_range = (cr_dst <= MAX_CLIST_REG);
    assign dst_perms = dst_reg_latched.word0_gt[57:48];  // Permission bits from destination GT
    assign src_perms = src_reg_latched.word0_gt[57:48];  // Permission bits from source GT
    assign dst_has_s_perm = dst_perms[PERM_S];
    assign src_has_b_perm = src_perms[PERM_B];           // GT.B must be TRUE to allow saving
    assign dst_limit = dst_reg_latched.word2_limit;
    assign dst_location = dst_reg_latched.word1_location;
    assign index_in_bounds = ({56'b0, index} < dst_limit);
    
    // ========================================================================
    // Memory Write Address Calculation
    // ========================================================================
    
    logic [63:0] write_addr;
    assign write_addr = dst_location + ({56'b0, index} << 3);  // index * 8 bytes
    
    // ========================================================================
    // Fault Latching
    // ========================================================================
    
    logic        fault_latched;
    fault_type_t fault_type_latched;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_latched <= 1'b0;
            fault_type_latched <= FAULT_NONE;
        end else if (state == SAVE_IDLE) begin
            fault_latched <= 1'b0;
            fault_type_latched <= FAULT_NONE;
        end else if (state == SAVE_CHECK_DST_READ && !dst_in_range) begin
            fault_latched <= 1'b1;
            fault_type_latched <= FAULT_PERM;  // Invalid destination register
        end else if (state == SAVE_CHECK_S_BOUNDS && !dst_has_s_perm) begin
            fault_latched <= 1'b1;
            fault_type_latched <= FAULT_PERM;  // Missing S permission on destination
        end else if (state == SAVE_CHECK_S_BOUNDS && dst_has_s_perm && !src_has_b_perm) begin
            fault_latched <= 1'b1;
            fault_type_latched <= FAULT_PERM;  // GT.B = FALSE (Golden Token cannot be saved)
        end else if (state == SAVE_CHECK_S_BOUNDS && dst_has_s_perm && src_has_b_perm && !index_in_bounds) begin
            fault_latched <= 1'b1;
            fault_type_latched <= FAULT_BOUNDS;  // Index out of bounds
        end
    end
    
    // ========================================================================
    // State Register
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= SAVE_IDLE;
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
            SAVE_IDLE: begin
                if (save_start)
                    next_state = SAVE_CHECK_DST_READ;
            end
            
            SAVE_CHECK_DST_READ: begin
                // Step 1: Verify CRd in 0-6 (combinational) AND initiate read
                // Range check happens combinationally, data available next cycle
                if (!dst_in_range)
                    next_state = SAVE_IDLE;  // Fault - destination out of range
                else
                    next_state = SAVE_LATCH_DST;
            end
            
            SAVE_LATCH_DST: begin
                // Data from destination read now valid, latch it
                // Also initiate source read
                next_state = SAVE_READ_SRC;
            end
            
            SAVE_READ_SRC: begin
                // Wait for source register data to become valid
                next_state = SAVE_CHECK_S_BOUNDS;
            end
            
            SAVE_CHECK_S_BOUNDS: begin
                // Step 2+3+4: Verify S permission, GT.B, AND bounds
                // Source data also latched in this cycle
                // Checks: dst.S (can save to), GT.B (can be saved), Index < Limit
                if (!dst_has_s_perm || !src_has_b_perm || !index_in_bounds)
                    next_state = SAVE_IDLE;  // Fault
                else
                    next_state = SAVE_WRITE_GT;
            end
            
            SAVE_WRITE_GT: begin
                // Step 4: Write GT to memory
                if (mem_wr_done)
                    next_state = SAVE_IDLE;
            end
            
            default: next_state = SAVE_IDLE;
        endcase
    end
    
    // ========================================================================
    // Register Read Control
    // ========================================================================
    // Read timing:
    //   IDLE (if save_start) -> CHECK_DST_READ: cr_rd_addr = cr_dst
    //   CHECK_DST_READ -> LATCH_DST: data valid, latch destination
    //   LATCH_DST -> READ_SRC: cr_rd_addr = cr_src
    //   READ_SRC -> CHECK_S_BOUNDS: data valid, latch source
    // ========================================================================
    
    always_comb begin
        cr_rd_addr = 4'd0;
        
        case (state)
            SAVE_IDLE: begin
                if (save_start)
                    cr_rd_addr = cr_dst;  // Start reading destination
            end
            SAVE_CHECK_DST_READ: begin
                cr_rd_addr = cr_dst;  // Continue reading destination
            end
            SAVE_LATCH_DST: begin
                cr_rd_addr = cr_src;  // Initiate source read
            end
            SAVE_READ_SRC: begin
                cr_rd_addr = cr_src;  // Continue reading source
            end
            default: cr_rd_addr = 4'd0;
        endcase
    end
    
    // ========================================================================
    // Memory Write Control
    // ========================================================================
    
    assign mem_wr_en = (state == SAVE_WRITE_GT);
    assign mem_wr_addr = write_addr;
    assign mem_wr_data = src_reg_latched.word0_gt;  // Write the Golden Token
    
    // ========================================================================
    // Output Signals
    // ========================================================================
    
    assign save_busy = (state != SAVE_IDLE);
    assign save_complete = (state == SAVE_WRITE_GT) && mem_wr_done;
    assign save_fault = fault_latched;
    assign fault_type = fault_type_latched;

endmodule
