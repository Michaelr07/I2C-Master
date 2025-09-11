import video_pkg::*;

module filt_border
    #(
        parameter video_pkg::vid_timing_t T = video_pkg::TIMING_VGA_640x480,
        parameter int TH = 1, //border thickness
        parameter logic [2:0] BC = 3'b000 // Border Color
    )
    (
        input logic clk, rst_n,
        input logic en,
        input pixel_t px_in,
        output pixel_t px_out,
        vid_sideband_if.sink    sb_in,
        vid_sideband_if.source  sb_out
    );
    
    localparam logic [video_pkg::RGBW-1:0] ON = '1;
    localparam logic [video_pkg::RGBW-1:0] OFF = '0;
    
    logic [video_pkg::RGBW-1 :0] r_r, r_g, r_b;
    wire in_border;
    
    assign in_border = ((sb_in.x < TH) || (sb_in.x >= T.H_ACTIVE-TH) || (sb_in.y < TH) || (sb_in.y >= T.V_ACTIVE-TH));

    always_comb begin
        r_r = px_in.R;
        r_g = px_in.G;
        r_b = px_in.B;
        
        if (in_border) begin
            unique case (BC)
                3'b000: {r_r,r_g,r_b}=    '1;               // white
                3'b001: {r_r,r_g,r_b} =   {ON, ON, OFF};    // yellow
                3'b010: {r_r,r_g,r_b} =   {OFF, ON, ON};    // cyan
                3'b011: {r_r,r_g,r_b} =   {OFF, ON, OFF};   // green
                3'b100: {r_r,r_g,r_b} =   {ON, OFF, ON};    // magenta
                3'b101: {r_r,r_g,r_b} =   {ON, OFF, OFF};   //red
                3'b110: {r_r,r_g,r_b} =   {OFF, OFF, ON};   //blue
                3'b111: {r_r,r_g,r_b} =   '0;               //black
                default: {r_r,r_g,r_b} =   '0;   
            endcase
        end   
    end
    
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
            
            if (sb_in.de) begin
                if (in_border && en)
                    px_out <= '{R:r_r, G:r_g, B:r_b};
                else 
                    px_out <= px_in;    
            end
            else
               px_out      <= '0;           
        end
    end
    
endmodule
