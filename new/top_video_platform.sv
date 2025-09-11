`timescale 1ns/1ps
`default_nettype none
import video_pkg::*;

module top_video_platform #(
  parameter vid_timing_t        T  = TIMING_VGA_640x480,
  parameter int                 TH = 2,
  parameter logic [2:0]         BC = 3'b101
) (
  // Board clocks & reset
  input  wire                  clk,
  input  wire                  rst_i,

  // User controls
  input  wire [4:0]            sw_i,
  input  wire                  btn_i,

  // Camera (OV7670)
  input  wire [7:0]            cam_d_i,
  input  wire                  cam_hs_i,
  input  wire                  cam_vs_i,
  input  wire                  cam_pclk_i,
  inout  tri                   cam_sda_io,   // open-drain
  inout  tri                   cam_scl_io,   // open-drain
  output wire                  cam_xclk_o,
  output wire                  cam_rstn_o,
  output wire                  cam_pwdn_o,

  // VGA output
  output wire [RGBW-1:0]       vga_r_o,
  output wire [RGBW-1:0]       vga_g_o,
  output wire [RGBW-1:0]       vga_b_o,
  output wire                  vga_hs_o,
  output wire                  vga_vs_o,

  // Status LED
  output wire                  locked_led_o
);


  // ---------------------------------------------------------------------------
  // Clocking / reset
  // ---------------------------------------------------------------------------
  logic        clk_pix;           // ~25 MHz for 640x480
  logic        mmcm_locked;
  logic [2:0]  pix_rst_sync;
  logic        rst_pix_n;

  // Generate: clk_pix + cam_xclk_o
  clk_wiz_pixcam u_clk (
    .clk_in1 (clk),
    .reset   (rst_i),
    .clk_out1(clk_pix),
    .clk_out2(cam_xclk_o),
    .locked  (mmcm_locked)
  );

  // Pixel-domain reset: async assert, sync deassert; re-assert on loss of lock
  always_ff @(posedge clk_pix or posedge rst_i) begin
    if (rst_i)           pix_rst_sync <= 3'b000;
    else if (!mmcm_locked) pix_rst_sync <= 3'b000;
    else                 pix_rst_sync <= {pix_rst_sync[1:0], 1'b1};
  end
  assign rst_pix_n    = pix_rst_sync[2];
  assign locked_led_o = mmcm_locked;

  // ---------------------------------------------------------------------------
  // CSR sync (switches/buttons to pixel domain)
  // ---------------------------------------------------------------------------
  localparam int N_SW  = $bits(sw_i);
  localparam int N_BTN = $bits(btn_i);

  logic [N_SW-1:0]  sw_sync;
  logic [N_BTN-1:0] btn_sync;

  csr_sync #(
    .N_SW (N_SW),
    .N_BTN(N_BTN)
  ) u_csr_sync (
    .clk      (clk_pix),
    .rst_n    (rst_pix_n),
    .async_sw (sw_i),
    .async_btn(btn_i),
    .sync_sw  (sw_sync),
    .sync_btn (btn_sync)
  );

  // Decode switches
  logic src_sel_req;   // 0 = TPG, 1 = camera
  logic en_grey_req;
  logic sel709_req;
  logic en_inv_req;
  logic en_border_req;

  assign src_sel_req    = sw_sync[0];
  assign en_grey_req    = sw_sync[1];
  assign sel709_req     = sw_sync[2];
  assign en_inv_req     = sw_sync[3];
  assign en_border_req  = sw_sync[4];

  // ---------------------------------------------------------------------------
  // VGA timing
  // ---------------------------------------------------------------------------
  localparam int XW = $clog2(T.H_ACTIVE);
  localparam int YW = $clog2(T.V_ACTIVE);

  logic [XW-1:0] x_pos;
  logic [YW-1:0] y_pos;
  logic          de;
  logic          hs_raw, vs_raw;

  vga_timing #(
    .T(T)
  ) u_timing (
    .PIX_CLK(clk_pix),
    .RST_N  (rst_pix_n),
    .DE     (de),
    .HSYNC  (hs_raw),
    .VSYNC  (vs_raw),
    .X_POS  (x_pos),
    .Y_POS  (y_pos)
  );

  // Sideband chain
  vid_sideband_if #(.XW(XW), .YW(YW)) sb0 ();
  vid_sideband_if #(.XW(XW), .YW(YW)) sb1 ();
  vid_sideband_if #(.XW(XW), .YW(YW)) sb2 ();
  vid_sideband_if #(.XW(XW), .YW(YW)) sb3 ();
  vid_sideband_if #(.XW(XW), .YW(YW)) sb4 ();

  // Drive sb0 from timing
  assign sb0.de  = de;
  assign sb0.x   = x_pos;
  assign sb0.y   = y_pos;
  assign sb0.sof = de && (x_pos == '0) && (y_pos == '0);
  assign sb0.eol = de && (x_pos == T.H_ACTIVE-1);

  // ---------------------------------------------------------------------------
  // Source select (TPG now; camera stubbed for future)
  // ---------------------------------------------------------------------------
  pixel_t px_src_tpg;
  pixel_t px_src_mux;

  tpg_colorbars #(.T(T)) u_tpg (
    .clk   (clk_pix),
    .rst_n (rst_pix_n),
    .px_out(px_src_tpg),
    .sb_in (sb0),
    .sb_out(sb1)
  );

  // TODO: camera capture puts px_src_cam and sb1 in place of TPG when src_sel_req == 1
  // pixel_t px_src_cam;  // from ov7670_capture (PCLK domain) + CDC/line FIFO into clk_pix domain

  // For now, force TPG (or simple 2:1 mux placeholder)
  assign px_src_mux = /* src_sel_req ? px_src_cam : */ px_src_tpg;

  // ---------------------------------------------------------------------------
  // Pixel pipeline: grayscale → invert → border
  // External latency accounting:
  //   GREY: 4 cycles, INV: 1 cycle, BORDER: 1 cycle, TPG: 1 cycle
  localparam int PIPE_LAT = 7;

  pixel_t px_grey, px_inv, px_border;

  filt_greyscale #(.XW(XW), .YW(YW)) u_grey (
    .clk   (clk_pix),
    .rst_n (rst_pix_n),
    .ctrl  ({sel709_req, en_grey_req}), // [1]=709 select, [0]=enable
    .px_in (px_src_mux),
    .px_out(px_grey),
    .sb_in (sb1),
    .sb_out(sb2)
  );

  filt_invert u_inv (
    .clk   (clk_pix),
    .rst_n (rst_pix_n),
    .en    (en_inv_req),
    .px_in (px_grey),
    .px_out(px_inv),
    .sb_in (sb2),
    .sb_out(sb3)
  );

  filt_border #(.T(T), .TH(TH), .BC(BC)) u_border (
    .clk   (clk_pix),
    .rst_n (rst_pix_n),
    .en    (en_border_req),
    .px_in (px_inv),
    .px_out(px_border),
    .sb_in (sb3),
    .sb_out(sb4)
  );

  // ---------------------------------------------------------------------------
  // Sync delay to match PIPE_LAT
  // ---------------------------------------------------------------------------
  logic [PIPE_LAT-1:0] hs_sr, vs_sr;
  always_ff @(posedge clk_pix or negedge rst_pix_n) begin
    if (!rst_pix_n) begin
      hs_sr <= '0;
      vs_sr <= '0;
    end else begin
      hs_sr <= {hs_sr[PIPE_LAT-2:0], hs_raw};
      vs_sr <= {vs_sr[PIPE_LAT-2:0], vs_raw};
    end
  end

  // ---------------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------------
  assign vga_hs_o = hs_sr[PIPE_LAT-1];
  assign vga_vs_o = vs_sr[PIPE_LAT-1];

  assign vga_r_o  = sb4.de ? px_border.R : '0;
  assign vga_g_o  = sb4.de ? px_border.G : '0;
  assign vga_b_o  = sb4.de ? px_border.B : '0;

  // Camera control defaults (safe)
  assign cam_rstn_o = 1'b1;   // hold out of reset
  assign cam_pwdn_o = 1'b0;   // sensor active

endmodule

`default_nettype wire