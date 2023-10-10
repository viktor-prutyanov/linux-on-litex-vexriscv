module exampleip0 (
    input clk,
    input rst,
    input en,
    output reg [31:0] cnt
);

always @(posedge clk) begin
    if (rst)
        cnt <= 32'b0;
    else if (en)
        cnt <= cnt + 1;
end

endmodule
