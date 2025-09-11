import video_pkg::*;

// Fully-registered grayscale with CSR-controlled enable and 601/709 select.
// Pipeline: S0 (capture) to S1 (mult) to S2 (sum+round) to S3 (mux/output)
// External latency is 4  cycles in both enabled and bypassed modes.
module filt_greyscale #(
  parameter int XW = 10,
  parameter int YW = 10
)(
  input  logic               clk,
  input  logic               rst_n,
  input  logic [1:0]         ctrl,    // ctrl[0]=en, ctrl[1]=use BT.709 (else BT.601)
  input  pixel_t             px_in,
  output var pixel_t         px_out,
  vid_sideband_if.sink       sb_in,
  vid_sideband_if.source     sb_out
);

  // ----- Fixed-point coeffs (Q0.15) -----
  localparam int  CW    = 16;
  localparam int  SHIFT = 15;
  localparam logic [SHIFT-1:0] ROUND = 1 << (SHIFT-1);

  // BT.601
  localparam logic [CW-1:0] KR_601 = 16'd9798;   // 0.2990 * 32768
  localparam logic [CW-1:0] KG_601 = 16'd19234;  // 0.5870 * 32768
  localparam logic [CW-1:0] KB_601 = 16'd3736;   // 0.1140 * 32768
  // BT.709 (rounded)
  localparam logic [CW-1:0] KR_709 = 16'd6966;   // 0.2126 * 32768
  localparam logic [CW-1:0] KG_709 = 16'd23436;  // 0.7152 * 32768
  localparam logic [CW-1:0] KB_709 = 16'd2366;   // 0.0722 * 32768

  localparam int IW = video_pkg::RGBW;
  
  //latching on start of frame
  logic sel709, en; 
 
  always_ff @(posedge clk or negedge rst_n)
    if(!rst_n) begin
         sel709 <= 1'b0;
         en     <= 1'b0;
    end
    else if (sb_in.sof) begin
        sel709 <= ctrl[1];
        en     <= ctrl[0];
    end 

  // ========================= S0: capture =========================
  logic [IW-1:0] r0, g0, b0;
  logic          de0, sof0, eol0;
  logic [XW-1:0] x0;
  logic [YW-1:0] y0;
  logic en0, sel0;
  pixel_t        px_b0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      en0 <= 1'b0; sel0 <= 1'b0;
      r0 <= '0; g0 <= '0; b0 <= '0;
      de0 <= 1'b0; sof0 <= 1'b0; eol0 <= 1'b0;
      x0 <= '0; y0 <= '0;
      px_b0 <= '0;
    end else begin
        en0  <= en;
        sel0 <= sel709;
        r0   <= px_in.R;
        g0   <= px_in.G;
        b0   <= px_in.B;
        de0  <= sb_in.de;
        sof0 <= sb_in.sof;
        eol0 <= sb_in.eol;
        x0   <= sb_in.x;
        y0   <= sb_in.y;
        px_b0 <= px_in;
    end
  end

// ========================= S1: mult =========================
  localparam int PWW  = IW + CW;      // product width
  localparam int SUMW = PWW + 2;      // headroom for pr+pg+pb

  logic [PWW-1:0] pr1, pg1, pb1;
  logic [IW-1:0]  r1,  g1,  b1;
  logic           de1, sof1, eol1;
  logic [XW-1:0]  x1;
  logic [YW-1:0]  y1;
  logic           en1; 

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pr1 <= '0; pg1 <= '0; pb1 <= '0;
      r1  <= '0; g1  <= '0; b1  <= '0;
      de1 <= 1'b0; sof1 <= 1'b0; eol1 <= 1'b0;
      x1  <= '0;   y1   <= '0;
      en1 <= 1'b0;
    end else begin
      // Multiply against per-frame-latched coeff set
      pr1 <= r0 * (sel0 ? KR_709 : KR_601);
      pg1 <= g0 * (sel0 ? KG_709 : KG_601);
      pb1 <= b0 * (sel0 ? KB_709 : KB_601);

      r1  <= r0; g1 <= g0; b1 <= b0;
      de1 <= de0; sof1 <= sof0; eol1 <= eol0;
      x1  <= x0;  y1   <= y0;
      en1 <= en0; 
    end
  end

    // ========================= S2: sum+round =========================
  logic [SUMW-1:0] sum2;
  logic [IW-1:0]   trunc_sum;
  logic [IW-1:0]   r2, g2, b2;
  logic            de2, sof2, eol2;
  logic [XW-1:0]   x2;
  logic [YW-1:0]   y2;
  logic            en2;

  localparam int RND = (1 << (SHIFT-1)); // 0x4000 for SHIFT=15
  
  localparam logic [SUMW-1:0] RND_W = {{(SUMW-SHIFT){1'b0}}, 1'b1, {(SHIFT-1){1'b0}}}; // 1<<(SHIFT-1)

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sum2 <= '0; trunc_sum <= '0;
      r2 <= '0; g2 <= '0; b2 <= '0;
      de2 <= 1'b0; sof2 <= 1'b0; eol2 <= 1'b0;
      x2  <= '0;   y2 <= '0;
      en2 <= 1'b0;
    end else begin
      sum2 <= pr1 + pg1 + pb1 + RND_W;
      // take bits [SHIFT +: IW] == (sum2 >> SHIFT) truncated to IW bits
      //trunc_sum   <= sum2[SHIFT +: IW];

      r2   <= r1; g2 <= g1; b2 <= b1;
      de2  <= de1; sof2 <= sof1; eol2 <= eol1;
      x2   <= x1;  y2 <= y1;
      en2  <= en1;
    end
  end

  // ========================= S3: mux/output =========================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        px_out      <= '0;
        sb_out.de   <= 1'b0;
        sb_out.sof  <= 1'b0;
        sb_out.eol  <= 1'b0;
        sb_out.x    <= '0;
        sb_out.y    <= '0;
    end else begin
      if (en2) begin
        px_out.R <= sum2[SHIFT +: IW]; px_out.G <= sum2[SHIFT +: IW]; px_out.B <= sum2[SHIFT +: IW];
      end else begin
        px_out.R <= r2; px_out.G <= g2; px_out.B <= b2;
      end
      sb_out.de <= de2; sb_out.sof <= sof2; sb_out.eol <= eol2;
      sb_out.x  <= x2;  sb_out.y   <= y2;
    end
  end

endmodule
