// ============================================================================
// CTMM LOADX/SAVEX - Load-Exclusive and Store-Exclusive Instructions
// ============================================================================
// Implements ARM-style exclusive access for atomic operations on capabilities
//
// LOADX CRd, [CRn, #offset]
//   - Load capability from namespace into CRd
//   - Set exclusive monitor for the namespace entry address
//   - Normal permission checks apply (L permission required)
//
// SAVEX CRs, [CRn, #offset], DRd
//   - Attempt to store capability CRs to namespace at offset
//   - If exclusive monitor still valid: store succeeds, DRd = 0
//   - If monitor cleared: store fails, DRd = 1, memory unchanged
//   - Monitor cleared after SAVEX (success or fail)
//
// Exclusive Monitor Logic:
//   - Per-thread flag tracking (namespace entry address, valid bit)
//   - Cleared when:
//     - Another thread does LOADX on same entry
//     - Another thread does SAVE/SAVEX on same entry
//     - This thread does SAVEX (success or fail)
// ============================================================================

module ctmm_loadx_savex
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        loadx_start,          // Start LOADX execution
    input  logic        savex_start,          // Start SAVEX execution
    input  logic [2:0]  cr_src,               // Source register for SAVEX (3-bit: CR0-CR7)
    input  logic [2:0]  cr_base,              // Base register (CRn) (3-bit: CR0-CR7)
    input  logic [2:0]  cr_dst,               // Destination register for LOADX (3-bit: CR0-CR7)
    input  logic [9:0]  offset,               // C-List index (10 bits: 1024 entries)
    input  logic [3:0]  result_dr,            // DR to store SAVEX result (0=success, 1=fail)
    input  logic [3:0]  thread_id,            // Current thread ID
    output logic        busy,                 // Operation in progress
    output logic        complete,             // Operation finished
    output logic        fault_valid,          // Operation caused a fault
    output fault_type_t fault_type,           // Type of fault
    
    // Capability register read interface
    output logic [3:0]  cr_rd_addr,           // Register to read
    input  capability_reg_t cr_rd_data,       // Full 256-bit register data
    
    // Capability register write interface
    output logic [3:0]  cr_wr_addr,           // Register to write
    output capability_reg_t cr_wr_data,       // Full 256-bit data to write
    output logic        cr_wr_en,             // Write enable
    
    // Data register write interface (for SAVEX result)
    output logic [3:0]  dr_wr_addr,           // DR to write
    output logic [63:0] dr_wr_data,           // 0 = success, 1 = fail
    output logic        dr_wr_en,             // Write enable
    
    // Memory interface (namespace access)
    output logic [63:0] mem_addr,             // Memory address
    output logic        mem_rd_en,            // Read enable
    input  logic [63:0] mem_rd_data,          // Read data
    input  logic        mem_rd_valid,         // Read data valid
    output logic [63:0] mem_wr_data,          // Write data
    output logic        mem_wr_en,            // Write enable
    
    // External clear interface (from other cores/threads)
    input  logic        ext_addr_match,       // External access to monitored address
    input  logic [31:0] ext_access_addr       // Address being accessed externally
);

    // ========================================================================
    // Per-Thread Exclusive Monitor Array
    // ========================================================================
    // Each thread has its own exclusive monitor to prevent false conflicts.
    // Maximum 16 threads supported (indexed by thread_id).
    // ========================================================================
    
    localparam int NUM_THREADS = 16;
    
    excl_monitor_t monitors [NUM_THREADS];
    logic [31:0] target_addr;
    
    // Calculate target address from base register + offset
    // 10-bit offset, shifted left by 5 (32 bytes per capability entry)
    assign target_addr = cr_rd_data.word1_location[31:0] + {17'h0, offset, 5'h0};
    
    // Current thread's monitor
    wire excl_monitor_t current_monitor = monitors[thread_id];
    
    // ========================================================================
    // State Machine
    // ========================================================================
    
    typedef enum logic [3:0] {
        IDLE,
        // LOADX states
        LOADX_READ_BASE,      // Read base register (CRn)
        LOADX_CHECK_PERM,     // Check L permission
        LOADX_CALC_ADDR,      // Calculate address and check bounds
        LOADX_FETCH_W0,       // Fetch Word 0 (GT)
        LOADX_FETCH_W1,       // Fetch Word 1 (Location)
        LOADX_FETCH_W2,       // Fetch Word 2 (Limit)
        LOADX_FETCH_W3,       // Fetch Word 3 (Seals)
        LOADX_SET_MONITOR,    // Set exclusive monitor
        LOADX_WRITE_DST,      // Write to destination register
        // SAVEX states
        SAVEX_READ_BASE,      // Read base register
        SAVEX_CHECK_MONITOR,  // Check if monitor still valid
        SAVEX_CHECK_PERM,     // Check S permission
        SAVEX_WRITE_MEM,      // Write capability to memory (if monitor valid)
        SAVEX_WRITE_RESULT,   // Write result to DR (0=success, 1=fail)
        // Common states
        COMPLETE,
        FAULT
    } state_t;
    
    state_t state, next_state;
    
    // ========================================================================
    // Latched Data
    // ========================================================================
    
    capability_reg_t base_reg_latched;
    capability_reg_t src_reg_latched;
    capability_reg_t fetched_cap;
    logic [31:0] addr_latched;
    logic monitor_was_valid;
    
    // Word fetch counter for LOADX
    logic [1:0] word_count;
    
    // ========================================================================
    // Per-Thread Exclusive Monitor Management
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_THREADS; i++) begin
                monitors[i].state <= EXCL_IDLE;
                monitors[i].addr <= 32'h0;
                monitors[i].thread_id <= 4'(i);
            end
        end else begin
            // External clear: check if any thread's monitor matches external access
            if (ext_addr_match) begin
                for (int i = 0; i < NUM_THREADS; i++) begin
                    if (monitors[i].state == EXCL_ACTIVE && 
                        monitors[i].addr == ext_access_addr) begin
                        monitors[i].state <= EXCL_CLEARED;
                    end
                end
            end
            
            // Set current thread's monitor on LOADX completion
            if (state == LOADX_SET_MONITOR) begin
                monitors[thread_id].state <= EXCL_ACTIVE;
                monitors[thread_id].addr <= addr_latched;
            end
            
            // Clear current thread's monitor on SAVEX completion
            if (state == SAVEX_WRITE_RESULT) begin
                monitors[thread_id].state <= EXCL_IDLE;
            end
        end
    end
    
    // Check if current thread's monitor is valid for this operation
    assign monitor_was_valid = (current_monitor.state == EXCL_ACTIVE) &&
                               (current_monitor.addr == addr_latched);
    
    // ========================================================================
    // Permission Check
    // ========================================================================
    
    logic has_l_perm;  // For LOADX
    logic has_s_perm;  // For SAVEX
    logic [9:0] base_perms;
    
    assign base_perms = base_reg_latched.word0_gt.perms[9:0];
    assign has_l_perm = base_perms[PERM_L];
    assign has_s_perm = base_perms[PERM_S];
    
    // ========================================================================
    // Bounds Check
    // ========================================================================
    
    logic bounds_ok;
    assign bounds_ok = ({20'h0, offset} < base_reg_latched.word2_limit[31:0]);
    
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
    // Latch Registers
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_reg_latched <= '0;
            src_reg_latched <= '0;
            addr_latched <= 32'h0;
            fetched_cap <= '0;
            word_count <= 2'h0;
        end else begin
            case (state)
                LOADX_READ_BASE, SAVEX_READ_BASE: begin
                    base_reg_latched <= cr_rd_data;
                    addr_latched <= target_addr;
                end
                LOADX_FETCH_W0: if (mem_rd_valid) fetched_cap.word0_gt <= mem_rd_data;
                LOADX_FETCH_W1: if (mem_rd_valid) fetched_cap.word1_location <= mem_rd_data;
                LOADX_FETCH_W2: if (mem_rd_valid) fetched_cap.word2_limit <= mem_rd_data;
                LOADX_FETCH_W3: if (mem_rd_valid) fetched_cap.word3_seals <= mem_rd_data;
                default: ;
            endcase
            
            // Latch source register for SAVEX
            if (state == SAVEX_CHECK_MONITOR) begin
                cr_rd_addr <= {1'b0, cr_src};
            end
        end
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
        end else if (state == LOADX_CHECK_PERM && !has_l_perm) begin
            fault_flag <= 1'b1;
            fault_latched <= FAULT_PERM_L;
        end else if (state == LOADX_CALC_ADDR && !bounds_ok) begin
            fault_flag <= 1'b1;
            fault_latched <= FAULT_BOUNDS;
        end else if (state == SAVEX_CHECK_PERM && !has_s_perm) begin
            fault_flag <= 1'b1;
            fault_latched <= FAULT_PERM_S;
        end
    end
    
    // ========================================================================
    // Next State Logic
    // ========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (loadx_start)
                    next_state = LOADX_READ_BASE;
                else if (savex_start)
                    next_state = SAVEX_READ_BASE;
            end
            
            // LOADX sequence
            LOADX_READ_BASE: next_state = LOADX_CHECK_PERM;
            
            LOADX_CHECK_PERM: begin
                if (!has_l_perm)
                    next_state = FAULT;
                else
                    next_state = LOADX_CALC_ADDR;
            end
            
            LOADX_CALC_ADDR: begin
                if (!bounds_ok)
                    next_state = FAULT;
                else
                    next_state = LOADX_FETCH_W0;
            end
            
            LOADX_FETCH_W0: if (mem_rd_valid) next_state = LOADX_FETCH_W1;
            LOADX_FETCH_W1: if (mem_rd_valid) next_state = LOADX_FETCH_W2;
            LOADX_FETCH_W2: if (mem_rd_valid) next_state = LOADX_FETCH_W3;
            LOADX_FETCH_W3: if (mem_rd_valid) next_state = LOADX_SET_MONITOR;
            
            LOADX_SET_MONITOR: next_state = LOADX_WRITE_DST;
            LOADX_WRITE_DST: next_state = COMPLETE;
            
            // SAVEX sequence
            SAVEX_READ_BASE: next_state = SAVEX_CHECK_MONITOR;
            SAVEX_CHECK_MONITOR: next_state = SAVEX_CHECK_PERM;
            
            SAVEX_CHECK_PERM: begin
                if (!has_s_perm)
                    next_state = FAULT;
                else if (monitor_was_valid)
                    next_state = SAVEX_WRITE_MEM;
                else
                    next_state = SAVEX_WRITE_RESULT;  // Skip write, just report fail
            end
            
            SAVEX_WRITE_MEM: next_state = SAVEX_WRITE_RESULT;
            SAVEX_WRITE_RESULT: next_state = COMPLETE;
            
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
            LOADX_READ_BASE, SAVEX_READ_BASE: cr_rd_addr = {1'b0, cr_base};
            SAVEX_CHECK_MONITOR: cr_rd_addr = {1'b0, cr_src};
            default: cr_rd_addr = 4'h0;
        endcase
    end
    
    // CR write (LOADX destination)
    assign cr_wr_addr = {1'b0, cr_dst};
    assign cr_wr_data = fetched_cap;
    assign cr_wr_en = (state == LOADX_WRITE_DST);
    
    // DR write (SAVEX result)
    assign dr_wr_addr = result_dr;
    assign dr_wr_data = monitor_was_valid ? 64'h0 : 64'h1;  // 0=success, 1=fail
    assign dr_wr_en = (state == SAVEX_WRITE_RESULT);
    
    // Memory interface
    always_comb begin
        mem_addr = 64'h0;
        mem_rd_en = 1'b0;
        mem_wr_data = 64'h0;
        mem_wr_en = 1'b0;
        
        case (state)
            LOADX_FETCH_W0: begin
                mem_addr = {32'h0, addr_latched};
                mem_rd_en = 1'b1;
            end
            LOADX_FETCH_W1: begin
                mem_addr = {32'h0, addr_latched} + 64'd8;
                mem_rd_en = 1'b1;
            end
            LOADX_FETCH_W2: begin
                mem_addr = {32'h0, addr_latched} + 64'd16;
                mem_rd_en = 1'b1;
            end
            LOADX_FETCH_W3: begin
                mem_addr = {32'h0, addr_latched} + 64'd24;
                mem_rd_en = 1'b1;
            end
            SAVEX_WRITE_MEM: begin
                mem_addr = {32'h0, addr_latched};
                mem_wr_data = src_reg_latched.word0_gt;
                mem_wr_en = 1'b1;
            end
            default: ;
        endcase
    end

endmodule
