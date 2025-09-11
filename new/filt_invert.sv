import video_pkg::*;

module filt_invert
    (
        input logic             clk, rst_n,
        input logic             en,
        input pixel_t           px_in,
        output pixel_t          px_out,
        vid_sideband_if.sink    sb_in,
        vid_sideband_if.source  sb_out
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_out      <= '0;
            sb_out.de   <= 1'b0;
            sb_out.sof  <= 1'b0;
            sb_out.eol  <= 1'b0;
            sb_out.x    <= '0;
            sb_out.y    <= '0;
        end
        else begin
            sb_out.de  <= sb_in.de;
            sb_out.sof <= sb_in.sof;
            sb_out.eol <= sb_in.eol;
            sb_out.x   <= sb_in.x;
            sb_out.y   <= sb_in.y;      
            
            if (sb_in.de)begin
                if(en) begin
                    px_out.R <= ~px_in.R;
                    px_out.G <= ~px_in.G;
                    px_out.B <= ~px_in.B;
                end
                else begin
                    px_out <= px_in;
                end
            end
            else begin
                px_out      <= '0; 
            end    
        end
    end
  
  
endmodule
