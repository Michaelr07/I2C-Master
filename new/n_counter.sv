`timescale 1ns / 1ps

module n_counter #(
    parameter int DIV = 500
) (
    input wire logic clk, rst_n,
    input wire logic en,
    output logic done
);

    logic [$clog2(DIV)-1:0] counter;
    
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            counter <= '0;
        else if (en) begin
           if (counter == DIV-1)
                counter <= '0;
           else 
                counter <= counter + 1;
        end
        else
            counter <= 0;
    end

    assign done = (counter == DIV-1);
    
endmodule
