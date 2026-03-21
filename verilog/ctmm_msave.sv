// ============================================================================
// CTMM mSave Micro-Routine - Shared Trusted Code for GT Saving
// ============================================================================
// Single trusted microcode for all GT saving operations.
//
// mSave Steps:
//   Step 1: Verify destination has B (Bind) flag set
//   Step 2: Verify destination has S (Save) permission
//   Step 3: Verify index < destination.word2_w2.limit_offset[15:0]
//   Step 4: Validate source GT against 4-word NS entry:
//             word0_gt25 (+0)  : GT[24:0] (25-bit token)
//             word1_location (+4): code pointer
//             word2_w2 (+8)    : limit_offset[20:0] | gt_seq[6:0] | spare[3:0]
//             word3_w3 (+12)   : crc[15:0] | g_bit | spare[14:0]
//           89-bit CRC over GT[24:0]+word1+word2_w2; match against word3_w3.crc
//           gt_seq from word2_w2.gt_seq must match src_gt_reg.gt_seq
//   Step 5: Write GT to Destination.Location + index*4 (32-bit words)
//
// NS entry stride: slot_id << 4 (×16 bytes = 4 words)
//
// FAULT conditions:
//   - Destination lacks B flag (FAULT_BIND)
//   - Destination lacks S permission (FAULT_PERM_S)
//   - index >= limit (FAULT_BOUNDS)
//   - gt_seq mismatch (FAULT_VERSION)
//   - CRC seal mismatch (FAULT_SEAL)
// ============================================================================

module ctmm_msave
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // Subroutine interface
    input  logic             sub_start,
    input  capability_reg_t  sub_dst_cap,  // Destination C-List capability
    input  logic [31:0]      sub_src_gt,   // Source Golden Token (32 bits)
    input  logic [15:0]      sub_index,    // C-List index
    output logic             sub_busy,
    output logic             sub_done,
    output logic             sub_fault,
    output fault_type_t      sub_fault_type,

    // CR15 (Namespace) interface
    input  capability_reg_t  cr15_namespace,

    // Memory write interface (32-bit)
    output logic [31:0] mem_wr_addr,
    output logic [31:0] mem_wr_data,
    output logic        mem_wr_en,
    input  logic        mem_wr_done,

    // Memory read interface (for NS entry validation)
    output logic [31:0] mem_rd_addr,
    output logic        mem_rd_en,
    input  logic [31:0] mem_rd_data,
    input  logic        mem_rd_valid
);

    // ========================================================================
    // State Machine
    // ========================================================================

    typedef enum logic [3:0] {
        SUB_IDLE,
        SUB_CHECK_BIND,
        SUB_CHECK_S,
        SUB_CHECK_BOUNDS,
        SUB_FETCH_NS_W0,
        SUB_FETCH_NS_W1,
        SUB_FETCH_NS_W2,
        SUB_FETCH_NS_W3,
        SUB_CHECK_VERSION,
        SUB_WRITE_GT,
        SUB_COMPLETE,
        SUB_FAULT
    } sub_state_t;

    sub_state_t state, next_state;

    // ========================================================================
    // Latched inputs
    // ========================================================================

    capability_reg_t dst_cap_reg;
    golden_token_t   src_gt_reg;
    logic [15:0]     index_reg;
    fault_type_t     fault_type_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dst_cap_reg    <= CR_NULL;
            src_gt_reg     <= GT_NULL;
            index_reg      <= 16'd0;
            fault_type_reg <= FAULT_NONE;
        end else begin
            if (state == SUB_IDLE && sub_start) begin
                dst_cap_reg    <= sub_dst_cap;
                src_gt_reg     <= sub_src_gt;
                index_reg      <= sub_index;
                fault_type_reg <= FAULT_NONE;
            end
            if (state == SUB_CHECK_BIND   && !dst_has_bind)    fault_type_reg <= FAULT_BIND;
            if (state == SUB_CHECK_S      && !dst_has_s_perm)  fault_type_reg <= FAULT_PERM_S;
            if (state == SUB_CHECK_BOUNDS && !index_in_bounds) fault_type_reg <= FAULT_BOUNDS;
            if (state == SUB_CHECK_VERSION) begin
                if (!gt_seq_match) fault_type_reg <= FAULT_VERSION;
                else if (!seal_ok) fault_type_reg <= FAULT_SEAL;
            end
        end
    end

    // ========================================================================
    // Checks
    // ========================================================================

    logic dst_has_bind;
    logic dst_has_s_perm;
    logic index_in_bounds;

    assign dst_has_bind    = dst_cap_reg.word0_gt.b_flag;
    assign dst_has_s_perm  = dst_cap_reg.word0_gt.perms[PERM_S];
    assign index_in_bounds = (index_reg < {11'h0, dst_cap_reg.word2_w2.limit_offset});

    // Write address: base + index*4 (32-bit words)
    logic [31:0] write_addr;
    assign write_addr = dst_cap_reg.word1_location + {14'h0, index_reg, 2'b00};

    // ========================================================================
    // Namespace entry address and 4-word entry validation
    // ========================================================================
    // NS stride: slot_id << 4  (4 words × 4 bytes = 16 bytes per entry)

    logic [31:0] ns_entry_addr;
    assign ns_entry_addr = cr15_namespace.word1_location +
                           ({16'h0, src_gt_reg.slot_id} << 4);

    logic [31:0] ns_w0_reg;    // word0_gt25: GT[24:0]
    logic [31:0] ns_w1_reg;    // word1_location
    logic [31:0] ns_w2_reg;    // word2_w2
    logic [31:0] ns_w3_reg;    // word3_w3

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_w0_reg <= 32'd0;
            ns_w1_reg <= 32'd0;
            ns_w2_reg <= 32'd0;
            ns_w3_reg <= 32'd0;
        end else begin
            if (state == SUB_FETCH_NS_W0 && mem_rd_valid) ns_w0_reg <= mem_rd_data;
            if (state == SUB_FETCH_NS_W1 && mem_rd_valid) ns_w1_reg <= mem_rd_data;
            if (state == SUB_FETCH_NS_W2 && mem_rd_valid) ns_w2_reg <= mem_rd_data;
            if (state == SUB_FETCH_NS_W3 && mem_rd_valid) ns_w3_reg <= mem_rd_data;
        end
    end

    // 89-bit CRC-16/CCITT over GT[24:0] (25 bits) + word1 (32 bits) + word2_w2 (32 bits)
    // Total: 89 input bits → 90 stages [0:89]
    // Input order: GT[24] first, then word1[31] ... word1[0], then word2[31] ... word2[0]
    logic [15:0] crc_stage [0:89];
    genvar gi;
    assign crc_stage[0] = CRC16_INIT;
    generate
        for (gi = 0; gi < 89; gi++) begin : crc_loop
            logic data_bit;
            logic top_bit;
            logic [15:0] shifted;
            if (gi < 25) begin
                assign data_bit = ns_w0_reg[24 - gi];      // GT[24:0], MSB first
            end else if (gi < 57) begin
                assign data_bit = ns_w1_reg[56 - gi];      // word1[31:0], MSB first
            end else begin
                assign data_bit = ns_w2_reg[88 - gi];      // word2[31:0], MSB first
            end
            assign top_bit         = crc_stage[gi][15] ^ data_bit;
            assign shifted         = {crc_stage[gi][14:0], 1'b0};
            assign crc_stage[gi+1] = shifted ^ (top_bit ? CRC16_POLY : 16'h0000);
        end
    endgenerate

    // View word3_w3 as a word3_t struct for field extraction
    word3_t ns_w3_view;
    assign ns_w3_view = ns_w3_reg;

    // View word2_w2 as a word2_t struct for gt_seq extraction
    word2_t ns_w2_view;
    assign ns_w2_view = ns_w2_reg;

    logic gt_seq_match;
    logic seal_ok;
    assign gt_seq_match = (src_gt_reg.gt_seq == ns_w2_view.gt_seq);
    assign seal_ok      = (crc_stage[89] == ns_w3_view.crc);

    // ========================================================================
    // State Register
    // ========================================================================

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= SUB_IDLE;
        else        state <= next_state;
    end

    // ========================================================================
    // Next State Logic
    // ========================================================================

    always_comb begin
        next_state = state;
        case (state)
            SUB_IDLE:
                if (sub_start) next_state = SUB_CHECK_BIND;
            SUB_CHECK_BIND:
                next_state = dst_has_bind ? SUB_CHECK_S : SUB_FAULT;
            SUB_CHECK_S:
                next_state = dst_has_s_perm ? SUB_CHECK_BOUNDS : SUB_FAULT;
            SUB_CHECK_BOUNDS:
                next_state = index_in_bounds ? SUB_FETCH_NS_W0 : SUB_FAULT;
            SUB_FETCH_NS_W0:
                if (mem_rd_valid) next_state = SUB_FETCH_NS_W1;
            SUB_FETCH_NS_W1:
                if (mem_rd_valid) next_state = SUB_FETCH_NS_W2;
            SUB_FETCH_NS_W2:
                if (mem_rd_valid) next_state = SUB_FETCH_NS_W3;
            SUB_FETCH_NS_W3:
                if (mem_rd_valid) next_state = SUB_CHECK_VERSION;
            SUB_CHECK_VERSION:
                if (!gt_seq_match || !seal_ok)
                    next_state = SUB_FAULT;
                else
                    next_state = SUB_WRITE_GT;
            SUB_WRITE_GT:
                if (mem_wr_done) next_state = SUB_COMPLETE;
            SUB_COMPLETE:
                next_state = SUB_IDLE;
            SUB_FAULT:
                next_state = SUB_IDLE;
            default: next_state = SUB_IDLE;
        endcase
    end

    // ========================================================================
    // Output Logic
    // ========================================================================

    assign sub_busy       = (state != SUB_IDLE);
    assign sub_done       = (state == SUB_COMPLETE);
    assign sub_fault      = (state == SUB_FAULT);
    assign sub_fault_type = fault_type_reg;

    // Memory read (for NS entry validation — 4-word entry at stride ×16)
    always_comb begin
        mem_rd_addr = 32'h0;
        mem_rd_en   = 1'b0;
        case (state)
            SUB_FETCH_NS_W0: begin mem_rd_addr = ns_entry_addr;      mem_rd_en = 1'b1; end
            SUB_FETCH_NS_W1: begin mem_rd_addr = ns_entry_addr + 4;  mem_rd_en = 1'b1; end
            SUB_FETCH_NS_W2: begin mem_rd_addr = ns_entry_addr + 8;  mem_rd_en = 1'b1; end
            SUB_FETCH_NS_W3: begin mem_rd_addr = ns_entry_addr + 12; mem_rd_en = 1'b1; end
            default: ;
        endcase
    end

    // Memory write
    assign mem_wr_en   = (state == SUB_WRITE_GT);
    assign mem_wr_addr = write_addr;
    assign mem_wr_data = src_gt_reg;

endmodule
