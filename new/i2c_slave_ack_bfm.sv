// tb/i2c_slave_ack_bfm.sv
`timescale 1ns/1ps
`default_nettype none

module i2c_slave_ack_bfm #(
  parameter logic [6:0] SLAVE_ADDR7 = 7'h21  // OV7670 = 0x21 (write=0 => 0x42)
)(
  input  wire logic clk,    // sim clock (not I2C clock)
  input  wire logic rst_n,

  inout  tri  sda,          // open-drain
  inout  tri  scl
);

  // open-drain drive
  logic sda_drv_low;
  assign sda = sda_drv_low ? 1'b0 : 1'bz;

  // 2-flop synchronizers of bus lines into clk domain
  logic sda_q, scl_q, sda_qq, scl_qq;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      sda_q  <= 1'b1; scl_q  <= 1'b1;
      sda_qq <= 1'b1; scl_qq <= 1'b1;
    end else begin
      sda_q  <= sda;   scl_q  <= scl;
      sda_qq <= sda_q; scl_qq <= scl_q;
    end
  end

  // edges / bus conditions
  wire scl_rise   = (scl_q==1'b1 && scl_qq==1'b0);
  wire scl_fall   = (scl_q==1'b0 && scl_qq==1'b1);
  wire start_cond = (scl_q==1'b1 && sda_q==1'b0 && sda_qq==1'b1); // START / repeated START
  wire stop_cond  = (scl_q==1'b1 && sda_q==1'b1 && sda_qq==1'b0); // STOP

  // simple BFM states
  typedef enum logic [2:0] {IDLE, START_WAIT, ADDR, DATA, READ, ACK} st_t;
  st_t          st;
  logic  [7:0]  shreg;
  logic  [3:0]  bit_idx;
  logic         ack_active;    // we are in the ACK bit and driving (if needed)
  logic         addr_byte;     // true during address byte reception
  logic         mode;          // 0=write (master -> slave), 1=read (slave -> master)

  // Optional: control to force NACK on a particular data byte (for negative tests)
  localparam int DATA_NACK_AT = -1; // set to e.g. 2 to NACK the 3rd data byte
  int data_byte_count;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st           <= IDLE;
      bit_idx      <= '0;
      shreg        <= '0;
      sda_drv_low  <= 1'b0;
      ack_active   <= 1'b0;
      addr_byte    <= 1'b0;
      mode         <= 1'b0;
      data_byte_count <= 0;
    end else begin
      // Default: release SDA each cycle; only pull low when explicitly ACKing
      sda_drv_low <= 1'b0;

      // Global preemption by repeated START or STOP (real devices must tolerate this)
      if (start_cond) begin
        st         <= START_WAIT;
        bit_idx    <= 4'd8;
        addr_byte  <= 1'b1;
        ack_active <= 1'b0;
        shreg      <= '0;
      end else if (stop_cond) begin
        st         <= IDLE;
        ack_active <= 1'b0;
        addr_byte  <= 1'b0;
      end else begin
        unique case (st)

          // -------------------------------------------------------------------
          // IDLE: wait for START
          // -------------------------------------------------------------------
          IDLE: begin
            if (/* no START here because handled globally above */ 1'b0) begin end
            // remain idle otherwise
          end
          START_WAIT: begin
            if(scl_fall) st  <= ADDR;
            else         st  <= START_WAIT;
          end
          // -------------------------------------------------------------------
          // ADDR: sample 7-bit address + R/W on scl rising edges
          // -------------------------------------------------------------------
          ADDR: begin
            if (scl_rise && bit_idx != 0) begin
                shreg[bit_idx-1] <= sda_qq;  // sample MSB..LSB
                bit_idx <= bit_idx - 4'd1;
            end else if (scl_fall && bit_idx == 0) begin
                mode       <= sda_qq;    // LSB just sampled is R/W bit
                st         <= ACK;
                ack_active <= 1'b1;      // we will actively ACK if address matches
            end    
          end

          // -------------------------------------------------------------------
          // DATA (write): sample data byte on scl rising edges
          // -------------------------------------------------------------------
          DATA: begin
             if (scl_rise && bit_idx != 0) begin
                shreg[bit_idx-1] <= sda_qq;  // sample MSB..LSB
                bit_idx <= bit_idx - 4'd1;
            end else if (scl_fall && bit_idx == 0) begin
                st         <= ACK;
                ack_active <= 1'b1;      // we will actively ACK if address matches
            end    
          end

          // -------------------------------------------------------------------
          // READ (slave would drive data): leave stubbed for now per your request
          // We won't drive data yet; master will likely NACK and stop in read tests.
          // -------------------------------------------------------------------
          READ: begin
            // For now, do nothing (release SDA). If master expects data, it will see 1's.
            // When you're ready to implement, you'll drive a data bit while SCL=0
            // and hold it stable while SCL=1, then tri-state for master ACK/NACK.
          end

          // -------------------------------------------------------------------
          // ACK: pull SDA low during the 9th clock if we want to ACK
          // -------------------------------------------------------------------
          ACK: begin
            if (ack_active) begin
              if (addr_byte) begin
                // Address ACK: pull low only if address matches
                sda_drv_low <= (shreg[7:1] == SLAVE_ADDR7);
              end else begin
                // Data ACK policy: ACK all data unless injecting a NACK for testing
                if (DATA_NACK_AT >= 0 && data_byte_count == DATA_NACK_AT)
                  sda_drv_low <= 1'b0; // leave high -> NACK
                else
                  sda_drv_low <= 1'b1; // ACK (pull low)
              end
            end

            // Release at end of ACK clock
            if (scl_fall) begin
              ack_active <= 1'b0;

              if (addr_byte) begin
                // Address phase just ended; decide next
                addr_byte <= 1'b0;
                if (shreg[7:1] != SLAVE_ADDR7) begin
                  // We NACKed address -> ignore until STOP or repeated START
                  st <= IDLE;
                end else begin
                  // Matched address -> branch by mode
                  bit_idx <= 4'd8;
                  if (mode == 1'b0) begin
                    st      <= DATA;               // WRITE: receive data
                    shreg   <= '0;
                  end else begin
                    st <= READ;               // READ: (stubbed for now)
                    
                  end
                end
              end else begin
                // Data ACK phase ended
                if (DATA_NACK_AT >= 0 && data_byte_count == DATA_NACK_AT) begin
                  // We NACKed this data byte: go idle and wait for STOP/START
                  st <= IDLE;
                end else begin
                  // ACKed data byte -> ready for next byte
                  data_byte_count <= data_byte_count + 1;
                  bit_idx         <= 4'd8;
                  st              <= DATA;
                end
              end
            end
          end

        endcase
      end
    end
  end
endmodule

`default_nettype wire
