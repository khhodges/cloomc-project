// ============================================================================
// CTMM LDM/STM - Load Multiple / Store Multiple Instructions
// ============================================================================
// Implements multiple register load/store operations for efficient
// context save/restore and block transfers.
//
// LDM CRn, {reg_list}
//   - Load multiple capability registers from consecutive C-List entries
//   - CRn = Base register pointing to C-List
//   - reg_list = 16-bit mask of registers to load (bit i = load CRi)
//
// STM CRn, {reg_list}
//   - Store multiple capability registers to consecutive C-List entries
//   - CRn = Base register pointing to C-List
//   - reg_list = 16-bit mask of registers to store (bit i = store CRi)
//
// Operation:
//   - Registers are loaded/stored in ascending order
//   - Uses mLoad/mSave for each register to enforce security checks:
//     - Permission validation (L for LDM, S for STM)
//     - Bounds checking (index < CRn.Limit)
//     - MAC validation for each capability
//   - This ensures LDM/STM cannot bypass capability security model
//
// Security: All operations go through mLoad/mSave trusted microcode
// ============================================================================

module ctmm_ldm_stm
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        ldm_start,            // Start LDM execution
    input  logic        stm_start,            // Start STM execution
    input  logic [2:0]  cr_base,              // Base register (CRn) - 3 bits for CR0-CR7
    input  logic [15:0] reg_list,             // Register list mask
    output logic        busy,                 // Operation in progress
    output logic        complete,             // Operation finished
    output logic        fault_valid,          // Operation caused a fault
    output fault_type_t fault_type,           // Type of fault
    
    // Capability register read interface
    output logic [3:0]  cr_rd_addr,           // Register to read
    input  capability_reg_t cr_rd_data,       // Full 256-bit register data
    
    // Capability register write interface  
    output logic [3:0]  cr_wr_addr,           // Register to write
    output capability_reg_t cr_wr_data,       // Data to write
    output logic        cr_wr_en,             // Write enable
    
    // CR15 (Namespace) interface for security checks
    input  capability_reg_t cr15_namespace,   // CR15 Namespace register
    
    // mLoad subroutine interface (for LDM security)
    output logic        mload_start,          // Start mLoad for current register
    output logic [3:0]  mload_src,            // Source register
    output logic [3:0]  mload_dst,            // Destination register
    output logic [9:0]  mload_index,          // C-List index
    input  logic        mload_busy,           // mLoad in progress
    input  logic        mload_done,           // mLoad complete
    input  logic        mload_fault,          // mLoad caused fault
    input  fault_type_t mload_fault_type,     // mLoad fault type
    
    // mSave subroutine interface (for STM security)
    output logic        msave_start,          // Start mSave for current register
    output logic [3:0]  msave_dst,            // Destination register
    output golden_token_t msave_gt,           // GT to save
    output logic [9:0]  msave_index,          // C-List index
    input  logic        msave_busy,           // mSave in progress
    input  logic        msave_done,           // mSave complete
    input  logic        msave_fault,          // mSave caused fault
    input  fault_type_t msave_fault_type      // mSave fault type
);

    // ========================================================================
    // State Machine
    // ========================================================================
    // Uses mLoad/mSave for each register to enforce full security checks
    // ========================================================================
    
    typedef enum logic [3:0] {
        IDLE,
        READ_BASE,            // Read base register
        PROCESS_REG,          // Identify next register to process
        START_MLOAD,          // Start mLoad for current register (LDM)
        WAIT_MLOAD,           // Wait for mLoad completion
        START_MSAVE,          // Start mSave for current register (STM)
        WAIT_MSAVE,           // Wait for mSave completion
        NEXT_REG,             // Move to next register
        COMPLETE,
        FAULT
    } state_t;
    
    state_t state, next_state;
    
    // ========================================================================
    // Control Registers
    // ========================================================================
    
    logic is_load;            // 1 = LDM, 0 = STM
    logic [15:0] reg_list_remaining;
    logic [3:0] current_reg;
    logic [9:0] current_index;  // Index into C-List
    capability_reg_t base_reg_latched;
    golden_token_t store_gt;
    
    // ========================================================================
    // Find Next Register
    // ========================================================================
    
    logic [3:0] next_reg;
    logic has_more_regs;
    
    always_comb begin
        next_reg = 4'd0;
        has_more_regs = 1'b0;
        for (int i = 0; i < 16; i++) begin
            if (reg_list_remaining[i] && !has_more_regs) begin
                next_reg = i[3:0];
                has_more_regs = 1'b1;
            end
        end
    end
    
    // Permission checks are done by mLoad/mSave - no need for separate check here
    
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
    // Control Logic
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_load <= 1'b0;
            reg_list_remaining <= 16'h0;
            current_reg <= 4'd0;
            current_index <= 10'd0;
            base_reg_latched <= '0;
            store_gt <= '0;
        end else begin
            case (state)
                IDLE: begin
                    if (ldm_start || stm_start) begin
                        is_load <= ldm_start;
                        reg_list_remaining <= reg_list;
                        current_index <= 10'd0;
                    end
                end
                
                READ_BASE: begin
                    base_reg_latched <= cr_rd_data;
                end
                
                PROCESS_REG: begin
                    current_reg <= next_reg;
                    reg_list_remaining[next_reg] <= 1'b0;
                end
                
                START_MSAVE: begin
                    // Latch the GT from the register to save
                    store_gt <= cr_rd_data.word0_gt;
                end
                
                NEXT_REG: begin
                    current_index <= current_index + 10'd1;
                end
                
                default: ;
            endcase
        end
    end
    
    // ========================================================================
    // Fault Latching (from mLoad/mSave)
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
        end else if (state == WAIT_MLOAD && mload_fault) begin
            fault_flag <= 1'b1;
            fault_latched <= mload_fault_type;
        end else if (state == WAIT_MSAVE && msave_fault) begin
            fault_flag <= 1'b1;
            fault_latched <= msave_fault_type;
        end
    end
    
    // ========================================================================
    // Next State Logic
    // ========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (ldm_start || stm_start)
                    next_state = READ_BASE;
            end
            
            READ_BASE: begin
                if (has_more_regs)
                    next_state = PROCESS_REG;
                else
                    next_state = COMPLETE;
            end
            
            PROCESS_REG: begin
                if (is_load)
                    next_state = START_MLOAD;
                else
                    next_state = START_MSAVE;
            end
            
            // LDM via mLoad
            START_MLOAD: next_state = WAIT_MLOAD;
            
            WAIT_MLOAD: begin
                if (mload_fault)
                    next_state = FAULT;
                else if (mload_done)
                    next_state = NEXT_REG;
            end
            
            // STM via mSave
            START_MSAVE: next_state = WAIT_MSAVE;
            
            WAIT_MSAVE: begin
                if (msave_fault)
                    next_state = FAULT;
                else if (msave_done)
                    next_state = NEXT_REG;
            end
            
            NEXT_REG: begin
                if (has_more_regs)
                    next_state = PROCESS_REG;
                else
                    next_state = COMPLETE;
            end
            
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
    
    // CR read address (3-bit CR expanded to 4-bit address)
    always_comb begin
        case (state)
            READ_BASE: cr_rd_addr = {1'b0, cr_base};
            START_MSAVE: cr_rd_addr = current_reg;  // current_reg is 4-bit but limited to 0-7
            default: cr_rd_addr = 4'h0;
        endcase
    end
    
    // CR write is handled by mLoad (writes directly to destination)
    assign cr_wr_addr = 4'h0;
    assign cr_wr_data = '0;
    assign cr_wr_en = 1'b0;
    
    // mLoad interface (for LDM) - expand 3-bit CR to 4-bit
    assign mload_start = (state == START_MLOAD);
    assign mload_src = {1'b0, cr_base};
    assign mload_dst = current_reg;
    assign mload_index = current_index;
    
    // mSave interface (for STM) - expand 3-bit CR to 4-bit
    assign msave_start = (state == START_MSAVE);
    assign msave_dst = {1'b0, cr_base};
    assign msave_gt = store_gt;
    assign msave_index = current_index;

endmodule
