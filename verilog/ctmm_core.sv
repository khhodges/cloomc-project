// ============================================================================
// CTMM Core - Church-Turing Meta-Machine Processor Core
// ============================================================================
// Top-level integration of all CTMM components
// Implements Kenneth James Hamer-Hodges' capability-based architecture
// ============================================================================

module ctmm_core
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Instruction Memory Interface
    output logic [31:0] imem_addr,        // Instruction address (NIA)
    input  logic [31:0] imem_data,        // Instruction data
    input  logic        imem_valid,       // Instruction valid
    
    // Namespace Memory Interface
    output logic [31:0] ns_addr,          // Namespace address
    output logic        ns_rd_en,         // Read enable
    input  namespace_entry_t ns_rd_data,  // Namespace entry data
    output namespace_entry_t ns_wr_data,  // Write data
    output logic        ns_wr_en,         // Write enable
    
    // C-List Memory Interface
    output logic [31:0] clist_addr,       // C-List address
    output logic        clist_rd_en,      // Read enable
    input  golden_token_t clist_rd_data,  // C-List entry (GT)
    output golden_token_t clist_wr_data,  // Write data
    output logic        clist_wr_en,      // Write enable
    
    // Data Memory Interface
    output logic [31:0] dmem_addr,        // Data address
    output logic        dmem_rd_en,       // Read enable
    input  logic [63:0] dmem_rd_data,     // Read data
    output logic [63:0] dmem_wr_data,     // Write data
    output logic        dmem_wr_en,       // Write enable
    
    // Boot Control
    input  logic        boot_start,       // Start boot sequence
    output boot_state_t boot_state,       // Current boot state
    output logic        boot_complete,    // Boot finished
    
    // GC Control
    input  logic        gc_start,         // Start GC cycle
    output logic        gc_busy,          // GC in progress
    output logic [31:0] gc_garbage_count, // Garbage entries found
    
    // Fault Output
    output fault_type_t fault,
    output logic        fault_valid,
    
    // Debug Interface
    output logic [31:0] nia,              // Next Instruction Address
    output condition_flags_t flags        // Current flags
);

    // ========================================================================
    // Internal Signals
    // ========================================================================
    
    // Register file signals
    logic [3:0]  cr_rd_addr, cr_wr_addr;
    golden_token_t cr_rd_data, cr_wr_data;
    logic        cr_wr_en;
    golden_token_t cr6_clist, cr7_nucleus, cr8_thread, cr15_namespace;
    golden_token_t cr6_wr_data, cr7_wr_data, cr8_wr_data, cr15_wr_data;
    logic        cr6_wr_en, cr7_wr_en, cr8_wr_en, cr15_wr_en;
    
    logic [3:0]  dr_rd_addr1, dr_rd_addr2, dr_wr_addr;
    logic [63:0] dr_rd_data1, dr_rd_data2, dr_wr_data;
    logic        dr_wr_en;
    
    condition_flags_t flags_internal, flags_in;
    logic        flags_wr_en;
    logic        clear_all;
    
    // Decoder signals
    logic        exec_enable, is_church_op, is_turing_op;
    church_opcode_t church_op;
    turing_opcode_t turing_op;
    logic [3:0]  cr_src, cr_dst;
    logic [7:0]  clist_index;
    logic [15:0] perm_mask;
    logic [3:0]  dr_src1, dr_src2, dr_dst;
    logic [15:0] immediate;
    logic        use_immediate;
    logic [15:0] branch_offset;
    fault_type_t decoder_fault;
    logic        decoder_fault_valid;
    
    // Permission checker signals
    golden_token_t perm_gt;
    logic [15:0] required_perms;
    logic        perm_check_valid;
    logic [31:0] access_index;
    logic [63:0] limit;
    logic        check_bounds;
    logic [63:0] calculated_mac, stored_mac;
    logic        check_mac;
    logic        perm_granted, bounds_ok, mac_valid, all_checks_pass;
    fault_type_t perm_fault;
    logic        perm_fault_valid;
    logic        g_bit_set, is_namespace_access;
    
    // GC unit signals
    logic [31:0] gc_ns_addr;
    logic        gc_ns_rd_en;
    golden_token_t gc_ns_wr_data;
    logic        gc_ns_wr_en;
    logic [31:0] gc_marked_count;
    logic        gc_done;
    logic        g_bit_reset;
    
    // NIA register
    logic [31:0] nia_reg;
    logic [31:0] nia_next;
    logic        nia_wr_en;
    
    // Boot state machine
    boot_state_t boot_state_reg;
    
    // ========================================================================
    // Register File Instantiation
    // ========================================================================
    
    ctmm_registers u_registers (
        .clk            (clk),
        .rst_n          (rst_n),
        .cr_rd_addr     (cr_rd_addr),
        .cr_rd_data     (cr_rd_data),
        .cr_wr_addr     (cr_wr_addr),
        .cr_wr_data     (cr_wr_data),
        .cr_wr_en       (cr_wr_en),
        .cr6_clist      (cr6_clist),
        .cr7_nucleus    (cr7_nucleus),
        .cr8_thread     (cr8_thread),
        .cr15_namespace (cr15_namespace),
        .cr6_wr_data    (cr6_wr_data),
        .cr6_wr_en      (cr6_wr_en),
        .cr7_wr_data    (cr7_wr_data),
        .cr7_wr_en      (cr7_wr_en),
        .cr8_wr_data    (cr8_wr_data),
        .cr8_wr_en      (cr8_wr_en),
        .cr15_wr_data   (cr15_wr_data),
        .cr15_wr_en     (cr15_wr_en),
        .dr_rd_addr1    (dr_rd_addr1),
        .dr_rd_data1    (dr_rd_data1),
        .dr_rd_addr2    (dr_rd_addr2),
        .dr_rd_data2    (dr_rd_data2),
        .dr_wr_addr     (dr_wr_addr),
        .dr_wr_data     (dr_wr_data),
        .dr_wr_en       (dr_wr_en),
        .flags          (flags_internal),
        .flags_in       (flags_in),
        .flags_wr_en    (flags_wr_en),
        .clear_all      (clear_all)
    );
    
    // ========================================================================
    // Instruction Decoder Instantiation
    // ========================================================================
    
    ctmm_decoder u_decoder (
        .clk            (clk),
        .rst_n          (rst_n),
        .instruction    (imem_data),
        .instr_valid    (imem_valid && boot_complete),
        .flags          (flags_internal),
        .exec_enable    (exec_enable),
        .is_church_op   (is_church_op),
        .is_turing_op   (is_turing_op),
        .church_op      (church_op),
        .cr_src         (cr_src),
        .cr_dst         (cr_dst),
        .clist_index    (clist_index),
        .perm_mask      (perm_mask),
        .turing_op      (turing_op),
        .dr_src1        (dr_src1),
        .dr_src2        (dr_src2),
        .dr_dst         (dr_dst),
        .immediate      (immediate),
        .use_immediate  (use_immediate),
        .branch_offset  (branch_offset),
        .fault          (decoder_fault),
        .fault_valid    (decoder_fault_valid)
    );
    
    // ========================================================================
    // Permission Checker Instantiation
    // ========================================================================
    
    ctmm_perm_check u_perm_check (
        .clk            (clk),
        .rst_n          (rst_n),
        .gt_in          (perm_gt),
        .required_perms (required_perms),
        .check_valid    (perm_check_valid),
        .access_index   (access_index),
        .limit          (limit),
        .check_bounds   (check_bounds),
        .calculated_mac (calculated_mac),
        .stored_mac     (stored_mac),
        .check_mac      (check_mac),
        .perm_granted   (perm_granted),
        .bounds_ok      (bounds_ok),
        .mac_valid      (mac_valid),
        .all_checks_pass(all_checks_pass),
        .fault_type     (perm_fault),
        .fault_valid    (perm_fault_valid),
        .g_bit_set      (g_bit_set),
        .is_namespace_access(is_namespace_access)
    );
    
    // ========================================================================
    // GC Unit Instantiation
    // ========================================================================
    
    ctmm_gc_unit u_gc_unit (
        .clk            (clk),
        .rst_n          (rst_n),
        .gc_start       (gc_start),
        .gc_mark_en     (1'b1),
        .gc_sweep_en    (1'b1),
        .gc_busy        (gc_busy),
        .gc_done        (gc_done),
        .ns_addr        (gc_ns_addr),
        .ns_rd_en       (gc_ns_rd_en),
        .ns_rd_data     (clist_rd_data), // Reuse C-List interface for GC
        .ns_wr_data     (gc_ns_wr_data),
        .ns_wr_en       (gc_ns_wr_en),
        .ns_start_addr  (32'h0),
        .ns_end_addr    (32'h1000),
        .marked_count   (gc_marked_count),
        .garbage_count  (gc_garbage_count),
        .access_addr    (clist_addr),
        .valid_key_access(all_checks_pass && is_church_op && (church_op == OP_LOAD)),
        .is_namespace_access(is_namespace_access),
        .g_bit_reset    (g_bit_reset)
    );
    
    // ========================================================================
    // NIA (Next Instruction Address) Register
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nia_reg <= 32'h0;
        end else if (clear_all) begin
            nia_reg <= 32'h0;
        end else if (nia_wr_en) begin
            nia_reg <= nia_next;
        end else if (exec_enable) begin
            nia_reg <= nia_reg + 32'h4; // Advance to next instruction
        end
    end
    
    assign imem_addr = nia_reg;
    assign nia = nia_reg;
    assign flags = flags_internal;
    
    // ========================================================================
    // Boot Sequence State Machine
    // ========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            boot_state_reg <= BOOT_IDLE;
        end else begin
            case (boot_state_reg)
                BOOT_IDLE: begin
                    if (boot_start)
                        boot_state_reg <= BOOT_FAULT_RST;
                end
                
                BOOT_FAULT_RST: begin
                    // Step 1: Clear all registers
                    boot_state_reg <= BOOT_LOAD_NS;
                end
                
                BOOT_LOAD_NS: begin
                    // Step 2: Load namespace into CR15
                    boot_state_reg <= BOOT_INIT_THRD;
                end
                
                BOOT_INIT_THRD: begin
                    // Step 3: Initialize thread in CR8
                    boot_state_reg <= BOOT_LOAD_NUC;
                end
                
                BOOT_LOAD_NUC: begin
                    // Step 4: Load nucleus
                    boot_state_reg <= BOOT_COMPLETE;
                end
                
                BOOT_COMPLETE: begin
                    // Stay in complete state
                    boot_state_reg <= BOOT_COMPLETE;
                end
                
                default: boot_state_reg <= BOOT_IDLE;
            endcase
        end
    end
    
    assign boot_state = boot_state_reg;
    assign boot_complete = (boot_state_reg == BOOT_COMPLETE);
    assign clear_all = (boot_state_reg == BOOT_FAULT_RST);
    
    // Boot sequence register loading
    always_comb begin
        cr15_wr_en = 1'b0;
        cr15_wr_data = GT_NULL;
        cr8_wr_en = 1'b0;
        cr8_wr_data = GT_NULL;
        cr6_wr_en = 1'b0;
        cr6_wr_data = GT_NULL;
        cr7_wr_en = 1'b0;
        cr7_wr_data = GT_NULL;
        
        case (boot_state_reg)
            BOOT_LOAD_NS: begin
                // Load namespace root GT into CR15
                cr15_wr_en = 1'b1;
                cr15_wr_data = '{
                    perms: PERM_MASK_M | PERM_MASK_L,  // M+L permissions
                    spare: 16'h0,
                    offset: 32'h0  // Offset 0 = Namespace
                };
            end
            
            BOOT_INIT_THRD: begin
                // Load thread GT into CR8
                cr8_wr_en = 1'b1;
                cr8_wr_data = '{
                    perms: PERM_MASK_M,  // M permission
                    spare: 16'h0,
                    offset: 32'h3  // Offset 3 = Kenneth thread
                };
            end
            
            BOOT_LOAD_NUC: begin
                // Load CR6 (Boot C-List) and CR7 (Nucleus)
                cr6_wr_en = 1'b1;
                cr6_wr_data = '{
                    perms: PERM_MASK_L | PERM_MASK_S | PERM_MASK_E,
                    spare: 16'h0,
                    offset: 32'h2  // Offset 2 = Boot C-List
                };
                cr7_wr_en = 1'b1;
                cr7_wr_data = '{
                    perms: PERM_MASK_X,
                    spare: 16'h0,
                    offset: 32'h1  // Offset 1 = Access.asm (Nucleus)
                };
            end
            
            default: begin
                // No writes
            end
        endcase
    end
    
    // ========================================================================
    // Fault Aggregation
    // ========================================================================
    
    always_comb begin
        if (decoder_fault_valid) begin
            fault = decoder_fault;
            fault_valid = 1'b1;
        end else if (perm_fault_valid) begin
            fault = perm_fault;
            fault_valid = 1'b1;
        end else begin
            fault = FAULT_NONE;
            fault_valid = 1'b0;
        end
    end
    
    // ========================================================================
    // Memory Interface Multiplexing
    // ========================================================================
    
    // Namespace/C-List address multiplexing
    assign clist_addr = gc_busy ? gc_ns_addr : 
                        (is_church_op ? {cr_rd_data.offset[23:0], clist_index} : 32'h0);
    assign clist_rd_en = gc_busy ? gc_ns_rd_en :
                         (exec_enable && is_church_op && (church_op == OP_LOAD));
    assign clist_wr_data = gc_busy ? gc_ns_wr_data : cr_rd_data;
    assign clist_wr_en = gc_busy ? gc_ns_wr_en :
                         (exec_enable && is_church_op && (church_op == OP_SAVE) && all_checks_pass);
    
    // Permission check setup for LOAD operation
    assign cr_rd_addr = cr_src;
    assign perm_gt = cr_rd_data;
    assign required_perms = (church_op == OP_LOAD) ? (PERM_MASK_L | PERM_MASK_M) :
                            (church_op == OP_SAVE) ? PERM_MASK_S :
                            (church_op == OP_CALL) ? PERM_MASK_E :
                            16'h0;
    assign perm_check_valid = exec_enable && is_church_op;
    assign access_index = {24'h0, clist_index};
    assign limit = ns_rd_data.word2_limit;
    assign check_bounds = 1'b1;
    assign calculated_mac = 64'h0; // Simplified - would compute MAC in real impl
    assign stored_mac = ns_rd_data.word3_seals;
    assign check_mac = 1'b0; // Disabled for now
    
    // CR write for LOAD operation
    assign cr_wr_addr = cr_dst;
    assign cr_wr_data = clist_rd_data;
    assign cr_wr_en = exec_enable && is_church_op && (church_op == OP_LOAD) && all_checks_pass;
    
    // DR interface for Turing operations
    assign dr_rd_addr1 = dr_src1;
    assign dr_rd_addr2 = dr_src2;
    assign dr_wr_addr = dr_dst;
    
    // ALU operation (simplified)
    always_comb begin
        dr_wr_data = 64'h0;
        dr_wr_en = 1'b0;
        flags_in = flags_internal;
        flags_wr_en = 1'b0;
        nia_wr_en = 1'b0;
        nia_next = nia_reg;
        
        if (exec_enable && is_turing_op) begin
            case (turing_op)
                OP_MOV: begin
                    dr_wr_data = use_immediate ? {48'h0, immediate} : dr_rd_data2;
                    dr_wr_en = 1'b1;
                end
                
                OP_ADD: begin
                    dr_wr_data = dr_rd_data1 + (use_immediate ? {48'h0, immediate} : dr_rd_data2);
                    dr_wr_en = 1'b1;
                    flags_in.Z = (dr_wr_data == 64'h0);
                    flags_in.N = dr_wr_data[63];
                    flags_wr_en = 1'b1;
                end
                
                OP_SUB: begin
                    dr_wr_data = dr_rd_data1 - (use_immediate ? {48'h0, immediate} : dr_rd_data2);
                    dr_wr_en = 1'b1;
                    flags_in.Z = (dr_wr_data == 64'h0);
                    flags_in.N = dr_wr_data[63];
                    flags_wr_en = 1'b1;
                end
                
                OP_AND: begin
                    dr_wr_data = dr_rd_data1 & (use_immediate ? {48'h0, immediate} : dr_rd_data2);
                    dr_wr_en = 1'b1;
                    flags_in.Z = (dr_wr_data == 64'h0);
                    flags_in.N = dr_wr_data[63];
                    flags_wr_en = 1'b1;
                end
                
                OP_ORR: begin
                    dr_wr_data = dr_rd_data1 | (use_immediate ? {48'h0, immediate} : dr_rd_data2);
                    dr_wr_en = 1'b1;
                    flags_in.Z = (dr_wr_data == 64'h0);
                    flags_in.N = dr_wr_data[63];
                    flags_wr_en = 1'b1;
                end
                
                OP_CMP: begin
                    logic [63:0] cmp_result;
                    cmp_result = dr_rd_data1 - (use_immediate ? {48'h0, immediate} : dr_rd_data2);
                    flags_in.Z = (cmp_result == 64'h0);
                    flags_in.N = cmp_result[63];
                    flags_wr_en = 1'b1;
                end
                
                OP_B: begin
                    nia_next = nia_reg + {{16{branch_offset[15]}}, branch_offset};
                    nia_wr_en = 1'b1;
                end
                
                default: begin
                    // No operation
                end
            endcase
        end
    end
    
    // Data memory interface (for data operations)
    assign dmem_addr = dr_rd_data1[31:0];
    assign dmem_rd_en = 1'b0;  // Simplified
    assign dmem_wr_data = dr_rd_data2;
    assign dmem_wr_en = 1'b0;  // Simplified
    
    // Namespace memory interface
    assign ns_addr = cr_rd_data.offset;
    assign ns_rd_en = perm_check_valid;
    assign ns_wr_data = '{default: '0};
    assign ns_wr_en = 1'b0;

endmodule
