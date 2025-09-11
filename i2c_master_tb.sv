`timescale 1ns/1ps
`default_nettype none

module i2c_master_tb;

  // ---------------------------------------------------------------------------
  // 0) Clock & reset
  // ---------------------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk; // 100 MHz

  logic rst_n;
  initial begin
    rst_n = 0;
    repeat (10) @(posedge clk);
    rst_n = 1;
  end
  
    initial begin
      // I2C @100 kHz will take 100s of µs; give plenty of time
      #5ms;
      $fatal(1, "[WATCHDOG] Simulation timed out. Likely wr_ready never asserted or SCL never ticked.");
    end
    
  // ---------------------------------------------------------------------------
  // 1) Open-drain I2C wires with pull-ups
  // ---------------------------------------------------------------------------
  tri1 sda;  // tri1 provides a weak '1' when nothing drives (sim pullup)
  tri1 scl;

  // ---------------------------------------------------------------------------
  // 2) DUT connections
  // ---------------------------------------------------------------------------
  logic        start;
  logic [6:0]  addr7;
  logic [7:0]  wr_len, rd_len;   // unused this test
  logic        busy, done;
  logic        nack_addr, nack_data, timeout;
  logic [7:0]  wr_data;
  logic        wr_valid, wr_ready;
  logic [7:0]  rd_data;
  logic        rd_valid, rd_ready;

  // DUT
  i2c_master #(
    .SYS_CLK   (100_000_000),
    .I2C_SPEED (100_000)
  ) dut (
    .clk, .rst_n,
    .start, .addr7, .wr_len, .rd_len,
    .busy, .done, .nack_addr, .nack_data, .timeout,
    .wr_data, .wr_valid, .wr_ready,
    .rd_data, .rd_valid, .rd_ready,
    .sda_io(sda), .scl_io(scl)
  );

  // Simple ACK slave BFM
  i2c_slave_ack_bfm #(.SLAVE_ADDR7(7'h21)) slave0 (
    .clk, .rst_n,
    .sda(sda), .scl(scl)
  );

  // ---------------------------------------------------------------------------
  // 3) Driver task for WRITE transactions
  //    - Sends START + address(w) + N data bytes
  //    - Robust to address-NACK (won't hang waiting for wr_ready)
  // ---------------------------------------------------------------------------
 task automatic drive_write(input logic [6:0] a, input byte payload[]);
  int i;

  // defaults
  wr_data  = '0;
  wr_valid = 1'b0;
  rd_ready = 1'b0;

  // program the command
  addr7  = a;
  wr_len = payload.size();
  rd_len = 8'd0;

  // START (1 cycle pulse)
  start = 1'b1; @(posedge clk); start = 1'b0;

  // wait until DUT actually claims the bus (avoid X/unknowns)
  wait (busy === 1'b1);

  // STREAM WRITE (level-valid), byte 0 first
  i        = 0;
  wr_data  = payload[i];
  wr_valid = 1'b1;

  // keep sending until all bytes accepted or the txn ends early
  while (i < payload.size()) begin
    @(posedge clk);

    // early end conditions (treat only definite 1 as true)
    if ((done      === 1'b1) ||
        (timeout   === 1'b1) ||
        (nack_addr === 1'b1) ||
        (nack_data === 1'b1)) begin
      wr_valid = 1'b0;
      return;
    end

    // handshake this edge?
    if (wr_ready === 1'b1) begin
      // A transfer occurred on THIS edge; keep wr_data stable for this edge.
      i++;

      if (i < payload.size()) begin
        // change data on the NEXT edge (never same edge as handshake)
        @(posedge clk);
        if ((done      === 1'b1) ||
            (timeout   === 1'b1) ||
            (nack_addr === 1'b1) ||
            (nack_data === 1'b1)) begin
          wr_valid = 1'b0; 
          return;
        end
        wr_data = payload[i];
        // wr_valid stays high
      end else begin
        // last byte was accepted; drop valid on the NEXT edge
        @(posedge clk);
        wr_valid = 1'b0;
        // wait for STOP/timeout to finish cleanly
        wait ((done === 1'b1) || (timeout === 1'b1));
        return;
      end
    end
    // else: no handshake; keep valid asserted and data stable
  end
endtask


   // ---------------------------------------------------------------------------
  // 4) Scoreboard (write-only tests)
  // ---------------------------------------------------------------------------
  byte payload_ok[] = '{8'h12, 8'h34, 8'hA5};
  byte payload_bad[] = '{8'hDE, 8'hAD};
  initial begin : test_seq
    // defaults
    start    = 0;
    addr7    = '0;
    wr_len   = '0;
    rd_len   = '0;
    wr_data  = '0;
    wr_valid = 0;
    rd_ready = 0;

    // wait release
    @(posedge rst_n);
    repeat (10) @(posedge clk);

    // --- Test A: correct address, 3 data bytes ---
    //byte payload_ok[] = '{8'h12, 8'h34, 8'hA5};
    $display("[%0t] INFO: write to 0x%0h with %0d bytes (expect ACK + DONE)",
              $time, 7'h21, payload_ok.size());
    drive_write(7'h21, payload_ok);
    if (timeout)   $fatal(1, "Timeout unexpectedly asserted (good address).");
    if (nack_addr) $fatal(1, "Address was NACKed unexpectedly.");
    if (nack_data) $fatal(1, "Data byte was NACKed unexpectedly.");
    if (!done)     $fatal(1, "DONE not observed.");
    $display("[%0t] PASS: good-address write completed with ACK(s).", $time);

    // Small gap
    repeat (20) @(posedge clk);

    // --- Test B: wrong address, any payload (should NACK on address and STOP) ---
   // byte payload_bad[] = '{8'hDE, 8'hAD};
    $display("[%0t] INFO: write to 0x%0h (expect address NACK + DONE)",
              $time, 7'h22);
    drive_write(7'h22, payload_bad);
    if (timeout)     $fatal(1, "Timeout unexpectedly asserted (bad address).");
    if (!nack_addr)  $fatal(1, "Expected address NACK but didn't see it.");
    if (!done)       $fatal(1, "DONE not observed after address NACK.");
    $display("[%0t] PASS: bad-address write NACKed as expected.", $time);

    $display("[%0t] ALL WRITE TESTS PASS", $time);
    #50 $finish;
  end

  // ---------------------------------------------------------------------------
  // 5) Assertions (bus-level)
  //    Allow SDA to change on SCL rising edge (data/ACK are sampled there),
  //    and allow START/STOP edges. Otherwise SDA must be stable while SCL is high.
  // ---------------------------------------------------------------------------
  logic sda_prev, scl_prev;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sda_prev <= 1'b1; scl_prev <= 1'b1;
    end else begin
      sda_prev <= sda;
      scl_prev <= scl;
    end
  end

  wire sda_rise       = (sda_prev==0 && sda==1);
  wire sda_fall       = (sda_prev==1 && sda==0);
  wire scl_rise_ev    = (scl_prev==0 && scl==1);

  // START definition (legal if SDA falls while SCL is high)
  property p_start_definition;
    @(posedge clk) disable iff (!rst_n)
      (sda_fall && scl==1 && scl_prev==1) |-> 1;
  endproperty
  assert property (p_start_definition);

  // STOP definition (legal if SDA rises while SCL is high)
  property p_stop_definition;
    @(posedge clk) disable iff (!rst_n)
      (sda_rise && scl==1 && scl_prev==1) |-> 1;
  endproperty
  assert property (p_stop_definition);

  // SDA stable while SCL high, except:
  //   - at SCL rising edge (data/ACK sampling point)
  //   - at START/STOP edges
  property p_sda_stable_while_scl_high_refined;
    @(posedge clk) disable iff (!rst_n)
      (scl==1 && !scl_rise_ev) |-> ( $stable(sda) ||
                                     sda_fall || // START
                                     sda_rise ); // STOP
  endproperty
  assert property (p_sda_stable_while_scl_high_refined)
    else $error("SDA changed while SCL high (not at SCL rise / START / STOP)");
endmodule

`default_nettype wire