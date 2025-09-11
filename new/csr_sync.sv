
module csr_sync             // 2 flipflop sync
#(
    parameter int N_SW = 4,
    parameter int N_BTN = 1
)
(
    input logic clk, rst_n,
    input logic [N_SW-1:0] async_sw,
    input logic [N_BTN-1:0] async_btn,
    output logic [N_SW-1:0] sync_sw,
    output logic [N_BTN-1:0] sync_btn
);
    
    logic [N_SW-1:0] sw_ff1, sw_ff2;
    logic [N_BTN-1:0] btn_ff1, btn_ff2;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sw_ff1  <= '0;
            sw_ff2  <= '0;
            btn_ff1 <= '0;
            btn_ff2 <= '0;        
        end
        else begin
            sw_ff1  <= async_sw;
            btn_ff1 <= async_btn;
            
            sw_ff2  <= sw_ff1;
            btn_ff2 <= btn_ff1;
        end  
    end
    
    assign sync_sw  = sw_ff2;
    assign sync_btn = btn_ff2;
endmodule
