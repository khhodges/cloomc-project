// ============================================================================
// CTMM mSave Micro-Routine - Shared Trusted Code for GT Saving
// ============================================================================
// This is the single trusted microcode for all GT saving operations.
// It minimizes the Trusted Computing Base (TCB) by providing verified
// save logic shared by SAVE instruction and CHANGE context switch.
//
// mSave Steps:
//   Step 1: Verify destination has S (Save) permission
//   Step 2: Verify GT.M || GT.B (machine-level bypasses B check)
//   Step 3: Verify Index < Destination.Limit
//   Step 4: Write GT to Destination.Location + Index*8
//
// Permission Rule:
//   - If GT.M = TRUE: Skip B check (machine-level capability)
//   - If GT.M = FALSE: Require GT.B = TRUE (software-level must be bound)
//
// FAULT conditions:
//   - Destination lacks S permission
//   - GT.M = FALSE AND GT.B = FALSE (software GT cannot be saved)
//   - Index >= Destination.Limit (out of bounds)
//
// Note: Caller is responsible for reading destination and source registers.
// mSave receives the already-read capability data.
// ============================================================================

module ctmm_msave
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // ========================================================================
    // Subroutine Interface - Called by SAVE, CHANGE
    // ========================================================================
    input  logic        sub_start,            // Start mSave
    input  capability_reg_t sub_dst_cap,      // Destination C-List capability (already read)
    input  logic [63:0] sub_src_gt,           // Source Golden Token to save
    input  logic [7:0]  sub_index,            // C-List index
    output logic        sub_busy,             // mSave in progress
    output logic        sub_done,             // mSave completed successfully
    output logic        sub_fault,            // mSave caused a fault
    output fault_type_t sub_fault_type,       // Type of fault
    
    // ========================================================================
    // Memory Write Interface
    // ========================================================================
    output logic [63:0] mem_wr_addr,          // Memory address to write
    output logic [63:0] mem_wr_data,          // Data to write (GT)
    output logic        mem_wr_en,            // Write enable
    input  logic        mem_wr_done           // Write complete acknowledgment
);

    // ========================================================================
    // State Machine
    // ========================================================================
    
    typedef enum logic [2:0] {
        SUB_IDLE,
        SUB_CHECK_S,          // Check destination has S permission
        SUB_CHECK_MB,         // Check GT.M || GT.B
        SUB_CHECK_BOUNDS,     // Check Index < Limit
        SUB_WRITE_GT,         // Write GT to memory
        SUB_COMPLETE,         // Signal completion
        SUB_FAULT             // Signal fault
    } sub_state_t;
    
    sub_state_t state, next_state;
    
    // ========================================================================
    // Latched Inputs
    // ========================================================================
    
    capability_reg_t dst_cap_reg;
    logic [63:0]     src_gt_reg;
    logic [7:0]      index_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dst_cap_reg <= '0;
            src_gt_reg <= '0;
            index_reg <= '0;
        end else if (state == SUB_IDLE && sub_start) begin
            dst_cap_reg <= sub_dst_cap;
            src_gt_reg <= sub_src_gt;
            index_reg <= sub_index;
        end
    end
    
    // ========================================================================
    // Permission and Bounds Check Logic
    // ========================================================================
    
    logic dst_has_s_perm;
    logic gt_has_m_perm;
    logic gt_has_b_perm;
    logic gt_can_be_saved;
    logic index_in_bounds;
    logic [9:0] dst_perms;
    logic [9:0] gt_perms;
    logic [63:0] dst_limit;
    logic [63:0] dst_location;
    
    assign dst_perms = dst_cap_reg.word0_gt[57:48];     // Destination permissions
    assign gt_perms = src_gt_reg[57:48];                // Source GT permissions
    assign dst_has_s_perm = dst_perms[PERM_S];
    assign gt_has_m_perm = gt_perms[PERM_M];
    assign gt_has_b_perm = gt_perms[PERM_B];
    assign gt_can_be_saved = gt_has_m_perm || gt_has_b_perm;  // M bypasses B check
    assign dst_limit = dst_cap_reg.word2_limit;
    assign dst_location = dst_cap_reg.word1_location;
    assign index_in_bounds = ({56'b0, index_reg} < dst_limit);
    
    // ========================================================================
    // Memory Write Address Calculation
    // ========================================================================
    
    logic [63:0] write_addr;
    assign write_addr = dst_location + ({56'b0, index_reg} << 3);  // index * 8 bytes
    
    // ========================================================================
    // Fault Type Register
    // ========================================================================
    
    fault_type_t fault_type_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_type_reg <= FAULT_NONE;
        end else if (state == SUB_IDLE) begin
            fault_type_reg <= FAULT_NONE;
        end else if (state == SUB_CHECK_S && !dst_has_s_perm) begin
            fault_type_reg <= FAULT_PERM;  // Missing S permission
        end else if (state == SUB_CHECK_MB && !gt_can_be_saved) begin
            fault_type_reg <= FAULT_PERM;  // GT.M=0 AND GT.B=0
        end else if (state == SUB_CHECK_BOUNDS && !index_in_bounds) begin
            fault_type_reg <= FAULT_BOUNDS;  // Index out of bounds
        end
    end
    
    // ========================================================================
    // State Register
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= SUB_IDLE;
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
            SUB_IDLE: begin
                if (sub_start)
                    next_state = SUB_CHECK_S;
            end
            
            SUB_CHECK_S: begin
                // Step 1: Verify destination has S permission
                if (!dst_has_s_perm)
                    next_state = SUB_FAULT;
                else
                    next_state = SUB_CHECK_MB;
            end
            
            SUB_CHECK_MB: begin
                // Step 2: Verify GT.M || GT.B (M bypasses B check)
                if (!gt_can_be_saved)
                    next_state = SUB_FAULT;
                else
                    next_state = SUB_CHECK_BOUNDS;
            end
            
            SUB_CHECK_BOUNDS: begin
                // Step 3: Verify Index < Limit
                if (!index_in_bounds)
                    next_state = SUB_FAULT;
                else
                    next_state = SUB_WRITE_GT;
            end
            
            SUB_WRITE_GT: begin
                // Step 4: Write GT to memory
                if (mem_wr_done)
                    next_state = SUB_COMPLETE;
            end
            
            SUB_COMPLETE: begin
                next_state = SUB_IDLE;
            end
            
            SUB_FAULT: begin
                next_state = SUB_IDLE;
            end
            
            default: next_state = SUB_IDLE;
        endcase
    end
    
    // ========================================================================
    // Memory Write Control
    // ========================================================================
    
    assign mem_wr_en = (state == SUB_WRITE_GT);
    assign mem_wr_addr = write_addr;
    assign mem_wr_data = src_gt_reg;  // Write the Golden Token
    
    // ========================================================================
    // Output Signals
    // ========================================================================
    
    assign sub_busy = (state != SUB_IDLE);
    assign sub_done = (state == SUB_COMPLETE);
    assign sub_fault = (state == SUB_FAULT);
    assign sub_fault_type = fault_type_reg;

endmodule
