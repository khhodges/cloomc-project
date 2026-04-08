module top(
    input clk,
    output led0,
    output led1,
    output led2,
    output led3,
    output led4,
    output led5
);
    reg [25:0] counter = 26'd0;

    always @(posedge clk)
        counter <= counter + 1'b1;

    assign led0 = ~counter[23];
    assign led1 = ~counter[24];
    assign led2 = ~counter[25];
    assign led3 = counter[23];
    assign led4 = counter[24];
    assign led5 = counter[25];
endmodule
