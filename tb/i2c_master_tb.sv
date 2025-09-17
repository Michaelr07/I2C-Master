`timescale 1ns/1ps
`default_nettype none

module i2c_master_tb;

    // ---------------------------------------------------------------------------
    //  Clock & reset
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
  //  Open-drain I2C wires with pull-ups
  // ---------------------------------------------------------------------------
    tri1 sda;  // tri1 provides a weak '1' when nothing drives (sim pullup)
    tri1 scl;

  // ---------------------------------------------------------------------------
  // DUT connections
  // ---------------------------------------------------------------------------
    logic        start;
    logic [6:0]  addr7;
    logic [7:0]  wr_len, rd_len;   
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
  //  Driver task for WRITE transactions
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
    
    // wait until DUT actually claims the bus
    wait (busy === 1'b1);
    
    // STREAM WRITE , byte 0 first
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

bit rd_valid_prev;

task automatic drive_read (input logic [6:0] a,
                            input int nbytes,
                            output byte rx[$]);
    int i;
    // defaults
    wr_data  = '0;
    wr_valid = 1'b0;
    rd_ready = 1'b0;
    
    rx = {};
    
    // program the command
    addr7  = a;
    wr_len = 8'd0;
    rd_len = byte'(nbytes);
    
    // issue START
    start = 1'b1; @(posedge clk); start = 1'b0;
    
    // wait until the DUT actually begins or terminates
    wait (busy || done || timeout);
    
    // if we already terminated, just exit
    if (done || timeout || nack_addr || nack_data) return;
    
    // STREAM READ
    rd_ready = 1'b1;
    
    // capture on rising edge of rd_valid to avoid double-count
    rd_valid_prev = 0;
    
    while (rx.size() < nbytes) begin 
        @(posedge clk);

        // end conditions
        if ( timeout || nack_addr || nack_data) begin
     
          wait(done);
            rd_ready = 1'b0;
            return;
        end

        // one-sample per rd_valid rising edge
        if (rd_valid && !rd_valid_prev) begin
          rx.push_back(rd_data);
        end
        rd_valid_prev = rd_valid;
    end

    // done or timeout
    wait (done || timeout);
    rd_ready = 1'b0;
endtask

// Combined transaction: START + addr(W) + payload  -> repeated START + addr(R) + read nbytes
task automatic drive_write_then_read (
    input  logic [6:0] a,
    input  byte        payload[],   // bytes to write
    input  int         nbytes,      // bytes to read
    output byte        rx[$]        // captured read data
);
    int  i;
    bit  rd_valid_prev;
    
    // defaults
    wr_data  = '0;
    wr_valid = 1'b0;
    rd_ready = 1'b0;
    rx = {};
    
    // program ONE command with both lengths
    addr7  = a;
    wr_len = payload.size();
    rd_len = byte'(nbytes);
    
    // START, DUT will do the repeated START internally
    start = 1'b1; @(posedge clk); start = 1'b0;
    
    // wait until the DUT actually begins or terminates
    wait (busy || done || timeout);
    
    // if we already terminated, bail early
    if (done || timeout || nack_addr || nack_data) return;
    
    // -------------------
    // WRITE streaming
    // -------------------
    i        = 0;
    if (payload.size() > 0) begin
        wr_data  = payload[i];
        wr_valid = 1'b1;
    
        while (i < payload.size()) begin
          @(posedge clk);
    
          // early end conditions
          if (timeout || nack_addr || nack_data) begin
            wait(done);
            wr_valid = 1'b0;
            return;
          end
    
          if (wr_ready === 1'b1) begin
            i++;
            if (i < payload.size()) begin
              @(posedge clk);
              if (done || timeout || nack_addr || nack_data) begin
                wr_valid = 1'b0;
                return;
              end
              wr_data = payload[i];
            end else begin
              // last byte accepted; drop valid on the NEXT edge
              @(posedge clk);
              wr_valid = 1'b0;
            end
          end
        end
    end

  // -------------------
  // READ streaming
  // -------------------
  // Be ready before the DUT enters the read phase (after the repeated START)
    rd_ready = 1'b1;
    rd_valid_prev = 0;

    while (rx.size() < nbytes) begin
        @(posedge clk);

        // end conditions
        if (timeout || nack_addr || nack_data) begin
            wait(done);
            rd_ready = 1'b0;
            return;
        end

        // capture once per rd_valid rising edge
        if (rd_valid && !rd_valid_prev) rx.push_back(rd_data);
        rd_valid_prev = rd_valid;
    end

      // wait for the DUT to complete with STOP (or timeout)
      wait (done || timeout);
      rd_ready = 1'b0;
endtask


    // ---------------------------------------------------------------------------
    //Scoreboard (write-only tests)
    // ---------------------------------------------------------------------------
    byte payload_ok[] = '{8'h12, 8'h34, 8'hA5};
    byte payload_bad[] = '{8'hDE, 8'hAD};
  
    byte got[$];
    byte exp3[] = '{8'hA0, 8'hA1, 8'hA2};
    byte exp4[] = '{8'hA0, 8'hA1, 8'hA2, 8'hA3};
    
    byte pay1[] = '{8'h00};     
    byte gotC[$];
    byte pay_bad[] = '{8'hEE};
    
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

//OTHER TESTS USING THE READ AND WRITE TASKS//

//    // --- Test A: correct address, 3 data bytes ---
//    //byte payload_ok[] = '{8'h12, 8'h34, 8'hA5};
//    $display("[%0t] INFO: write to 0x%0h with %0d bytes (expect ACK + DONE)",
//              $time, 7'h21, payload_ok.size());
//    drive_write(7'h21, payload_ok);
//    if (timeout)   $fatal(1, "Timeout unexpectedly asserted (good address).");
//    if (nack_addr) $fatal(1, "Address was NACKed unexpectedly.");
//    if (nack_data) $fatal(1, "Data byte was NACKed unexpectedly.");
//    if (!done)     $fatal(1, "DONE not observed.");
//    $display("[%0t] PASS: good-address write completed with ACK(s).", $time);

//    // Small gap
//    repeat (20) @(posedge clk);

//  // Test R1: correct address, read 3 bytes
//  $display("[%0t] INFO: READ 3 bytes from 0x%0h (expect A0,A1,A2 + DONE)", $time, 7'h21);
//  drive_read(7'h21, 3, got);
//  if (timeout)   $fatal(1, "Timeout asserted unexpectedly (good address, read).");
//  if (nack_addr) $fatal(1, "Address was NACKed unexpectedly on read.");
//  if (nack_data) $fatal(1, "Data was NACKed unexpectedly during read.");
//  if (!done)     $fatal(1, "DONE not observed after read.");
//  if (got.size()!=3) $fatal(1, "Expected 3 bytes, got %0d.", got.size());
//  foreach (got[i]) if (got[i] !== exp3[i]) $fatal(1, "Byte %0d mismatch: got %02h exp %02h", i, got[i], exp3[i]);
//  $display("[%0t] PASS: read 3 matched A0,A1,A2", $time);

//  // Small gap
//  repeat (20) @(posedge clk);

//  // Test R2: wrong address, try to read 2 bytes (should addr-NACK and return no data)
//  $display("[%0t] INFO: READ from 0x%0h (expect addr NACK + DONE, no data)", $time, 7'h22);
//  drive_read(7'h22, 2, got);
//  if (timeout)    $fatal(1, "Timeout asserted unexpectedly (bad address read).");
//  if (!nack_addr) $fatal(1, "Expected address NACK on read but didn't see it.");
//  if (!done)      $fatal(1, "DONE not observed after address NACK on read.");
//  if (got.size()!=0) $fatal(1, "Expected 0 data bytes on address NACK, got %0d.", got.size());
//  $display("[%0t] PASS: bad-address read NACKed as expected.", $time);

//  // Small gap
//  repeat (20) @(posedge clk);

//  // Test R3: correct address, read 4 bytes
//  $display("[%0t] INFO: READ 4 bytes from 0x%0h (expect A0..A3 + DONE)", $time, 7'h21);
//  drive_read(7'h21, 4, got);
//  if (timeout)   $fatal(1, "Timeout asserted unexpectedly (good address, read4).");
//  if (nack_addr) $fatal(1, "Address was NACKed unexpectedly on read4.");
//  if (nack_data) $fatal(1, "Data was NACKed unexpectedly during read4.");
//  if (!done)     $fatal(1, "DONE not observed after read4.");
//  if (got.size()!=4) $fatal(1, "Expected 4 bytes, got %0d.", got.size());
//  foreach (got[i]) if (got[i] !== exp4[i]) $fatal(1, "Byte %0d mismatch: got %02h exp %02h", i, got[i], exp4[i]);
//  $display("[%0t] PASS: read 4 matched A0..A3", $time);
  
//  repeat (20) @(posedge clk);
  
    // --- C1: Combined write(1) then read(3) on good address ---
    $display("[%0t] INFO: COMBINED: write1 then read3 @0x%0h", $time, 7'h21);
    drive_write_then_read(7'h21, pay1, 3, gotC);
    if (timeout)   $fatal(1, "Timeout in combined tx.");
    if (nack_addr) $fatal(1, "Address NACK in combined tx.");
    if (nack_data) $fatal(1, "Data NACK in combined tx (write phase).");
    if (!done)     $fatal(1, "DONE not observed in combined tx.");
    if (gotC.size()!=3) $fatal(1, "Expected 3 bytes, got %0d.", gotC.size());
    // Our BFM returns A0,A1,A2 from start
    foreach (gotC[i]) if (gotC[i] !== exp3[i]) $fatal(1, "C1 mismatch byte %0d: got %02h exp %02h", i, gotC[i], exp3[i]);
    $display("[%0t] PASS: combined write1/read3 OK", $time);
    
    // Small gap
    repeat (20) @(posedge clk);
    
    // --- C2: Combined with wrong address -> expect addr NACK and no data
    gotC = {};
    $display("[%0t] INFO: COMBINED: write1/read2 bad address @0x%0h", $time, 7'h22);
    drive_write_then_read(7'h22, pay_bad, 2, gotC);
    if (timeout)    $fatal(1, "Timeout on bad-address combined.");
    if (!nack_addr) $fatal(1, "Expected address NACK on combined.");
    if (!done)      $fatal(1, "DONE not observed after address NACK (combined).");
    if (gotC.size()!=0) $fatal(1, "Expected 0 bytes after address NACK, got %0d.", gotC.size());
    $display("[%0t] PASS: combined bad-address NACK OK", $time);
    

    $display("[%0t] ALL TESTS PASS", $time);
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
