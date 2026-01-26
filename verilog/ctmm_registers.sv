// ============================================================================
// CTMM Register File - Context and Data Registers
// ============================================================================
// Church Registers (CR0-CR15): Hold Golden Tokens for capability access
//   CR0-CR7:  General purpose capability registers
//   CR6:      Current C-List
//   CR7:      Nucleus (kernel capability)
//   CR8:      Thread identity
//   CR15:     Namespace root
// Turing Registers (DR0-DR15): Hold 64-bit data values
// ============================================================================

module ctmm_registers
    import ctmm_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // Context Register Interface (Church)
    input  logic [3:0]  cr_rd_addr,      // Read address
    output golden_token_t cr_rd_data,    // Read data
    input  logic [3:0]  cr_wr_addr,      // Write address
    input  golden_token_t cr_wr_data,    // Write data
    input  logic        cr_wr_en,        // Write enable
    
    // Special Context Registers
    output golden_token_t cr6_clist,     // Current C-List
    output golden_token_t cr7_nucleus,   // Nucleus
    output golden_token_t cr8_thread,    // Thread identity
    output golden_token_t cr15_namespace,// Namespace root
    
    input  golden_token_t cr6_wr_data,
    input  logic        cr6_wr_en,
    input  golden_token_t cr7_wr_data,
    input  logic        cr7_wr_en,
    input  golden_token_t cr8_wr_data,
    input  logic        cr8_wr_en,
    input  golden_token_t cr15_wr_data,
    input  logic        cr15_wr_en,
    
    // Data Register Interface (Turing)
    input  logic [3:0]  dr_rd_addr1,     // Read address 1
    output logic [63:0] dr_rd_data1,     // Read data 1
    input  logic [3:0]  dr_rd_addr2,     // Read address 2
    output logic [63:0] dr_rd_data2,     // Read data 2
    input  logic [3:0]  dr_wr_addr,      // Write address
    input  logic [63:0] dr_wr_data,      // Write data
    input  logic        dr_wr_en,        // Write enable
    
    // Condition Flags
    output condition_flags_t flags,
    input  condition_flags_t flags_in,
    input  logic        flags_wr_en,
    
    // Clear all registers (boot step 1)
    input  logic        clear_all
);

    // ========================================================================
    // Context Registers (CR0-CR7)
    // ========================================================================
    
    golden_token_t context_regs [0:7];
    
    // Read from CR0-CR7 or special registers
    always_comb begin
        case (cr_rd_addr)
            4'd0, 4'd1, 4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7:
                cr_rd_data = context_regs[cr_rd_addr[2:0]];
            4'd8:  cr_rd_data = cr8_thread;
            4'd15: cr_rd_data = cr15_namespace;
            default: cr_rd_data = GT_NULL;
        endcase
    end
    
    // Write to CR0-CR7
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            for (int i = 0; i < 8; i++) begin
                context_regs[i] <= GT_NULL;
            end
        end else if (cr_wr_en && cr_wr_addr < 4'd8) begin
            context_regs[cr_wr_addr[2:0]] <= cr_wr_data;
        end
    end
    
    // ========================================================================
    // Special Context Registers
    // ========================================================================
    
    // CR6: Current C-List
    golden_token_t cr6_reg;
    assign cr6_clist = cr6_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            cr6_reg <= GT_NULL;
        end else if (cr6_wr_en) begin
            cr6_reg <= cr6_wr_data;
        end else if (cr_wr_en && cr_wr_addr == 4'd6) begin
            cr6_reg <= cr_wr_data;
        end
    end
    
    // CR7: Nucleus
    golden_token_t cr7_reg;
    assign cr7_nucleus = cr7_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            cr7_reg <= GT_NULL;
        end else if (cr7_wr_en) begin
            cr7_reg <= cr7_wr_data;
        end else if (cr_wr_en && cr_wr_addr == 4'd7) begin
            cr7_reg <= cr_wr_data;
        end
    end
    
    // CR8: Thread Identity
    golden_token_t cr8_reg;
    assign cr8_thread = cr8_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            cr8_reg <= GT_NULL;
        end else if (cr8_wr_en) begin
            cr8_reg <= cr8_wr_data;
        end
    end
    
    // CR15: Namespace Root
    golden_token_t cr15_reg;
    assign cr15_namespace = cr15_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            cr15_reg <= GT_NULL;
        end else if (cr15_wr_en) begin
            cr15_reg <= cr15_wr_data;
        end
    end
    
    // ========================================================================
    // Data Registers (DR0-DR15)
    // ========================================================================
    
    logic [63:0] data_regs [0:15];
    
    // Dual-port read
    assign dr_rd_data1 = data_regs[dr_rd_addr1];
    assign dr_rd_data2 = data_regs[dr_rd_addr2];
    
    // Write
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            for (int i = 0; i < 16; i++) begin
                data_regs[i] <= 64'h0;
            end
        end else if (dr_wr_en) begin
            data_regs[dr_wr_addr] <= dr_wr_data;
        end
    end
    
    // ========================================================================
    // Condition Flags
    // ========================================================================
    
    condition_flags_t flags_reg;
    assign flags = flags_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_all) begin
            flags_reg <= '{N: 1'b0, Z: 1'b0, C: 1'b0, V: 1'b0};
        end else if (flags_wr_en) begin
            flags_reg <= flags_in;
        end
    end

endmodule
