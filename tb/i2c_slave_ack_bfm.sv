// tb/i2c_slave_ack_bfm.sv
`timescale 1ns/1ps
`default_nettype none

module i2c_slave_ack_bfm #(
  parameter logic [6:0] SLAVE_ADDR7 = 7'h21,
  parameter int DATA_NACK_AT = -1   // -1 = never; else NACK that (0-based) data byte
)(
  input  wire logic clk,
  input  wire logic rst_n,

  inout  tri  sda,
  inout  tri  scl
);

  // open-drain drive (pull low when sda_drv_low=1, else release)
  logic sda_drv_low;
  assign sda = sda_drv_low ? 1'b0 : 1'bz;

  // 2FF sync of pins into clk domain
  logic sda_q, scl_q, sda_qq, scl_qq;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      sda_q<=1'b1; sda_qq<=1'b1;
      scl_q<=1'b1; scl_qq<=1'b1;
    end else begin
      sda_q<=sda;   sda_qq<=sda_q;
      scl_q<=scl;   scl_qq<=scl_q;
    end
  end

  byte read_mem[] = '{8'hA0, 8'hA1, 8'hA2, 8'hA3}; // example data
  int  rd_idx;
  
  logic scl_rise, scl_fall, start_cond, stop_cond;
  // edges and bus conditions
  assign scl_rise   = (scl_q==1'b1 && scl_qq==1'b0);
  assign scl_fall   = (scl_q==1'b0 && scl_qq==1'b1);
  assign start_cond = (scl_q==1'b1 && sda_q==1'b0 && sda_qq==1'b1);
  assign stop_cond  = (scl_q==1'b1 && sda_q==1'b1 && sda_qq==1'b0);

  typedef enum logic [2:0] {IDLE, START_WAIT, ADDR, DATA, READ, READ_ACK, ACK} st_t;
  st_t          st;

  logic  [7:0]  shreg, tx_reg;
  logic  [3:0]  bit_idx;        // counts 7..0
  logic         addr_byte;      // true during address/RW reception
  logic         mode_read;      // 0=write, 1=read
  int           data_byte_count;

  // ACK
  logic         ack_active;     // we are in the ACK bit (drive low while true)
  logic         ack_drive_low;  // latched decision for this ACK bit

  function automatic logic should_nack_this_data(int idx);
    return (DATA_NACK_AT >= 0) && (idx == DATA_NACK_AT);
  endfunction

  // combinational helper: address match
  function automatic logic addr_match(input logic [7:0] a);
    return (a[7:1] == SLAVE_ADDR7);
  endfunction

  // Sequential FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st              <= IDLE;
      bit_idx         <= 4'd0;
      shreg           <= '0;
      tx_reg        <= '0;
      rd_idx        <= '0;
      sda_drv_low     <= 1'b0;
      ack_active      <= 1'b0;
      ack_drive_low   <= 1'b0;
      addr_byte       <= 1'b0;
      mode_read       <= 1'b0;
      data_byte_count <= 0;
    end else begin
      // default: only drive during an ACK bit; keep level stable across SCL high
      sda_drv_low <= (ack_active && ack_drive_low);

      if (start_cond) begin
        st            <= START_WAIT;
        bit_idx       <= 4'd8;      // expect to shift 8 bits
        addr_byte     <= 1'b1;
        ack_active    <= 1'b0;
        ack_drive_low <= 1'b0;
        shreg         <= '0;
        tx_reg        <= '0;
        rd_idx        <= '0;
        
      end else if (stop_cond) begin
        st            <= IDLE;
        ack_active    <= 1'b0;
        ack_drive_low <= 1'b0;
        addr_byte     <= 1'b0;
      end else begin
        unique case (st)

          // Wait for SCL to go low after START
          START_WAIT: begin
            if (scl_fall) st <= ADDR;
          end

          // ------------------------------------------------------------------
          // Address: shift LSB on each SCL rising edge (R/W is bit0)
          // After last bit sampled (bit_idx==0 at rising edge), wait for
          // SCL falling edge to arm ACK and decide address ACK/NACK.
          // ------------------------------------------------------------------
          ADDR: begin
            if (scl_rise) begin
              shreg <= {shreg[6:0], sda_qq};
              if (bit_idx != 0) bit_idx <= bit_idx - 4'd1;
            end
            // after the last data bit's HIGH period ends, prepare ACK on next LOW
            if (bit_idx == 0 && scl_fall) begin
              mode_read     <= sda_qq;              // R/W bit we just sampled
              ack_active    <= 1'b1;                // enter ACK bit
              ack_drive_low <= addr_match(shreg);   // ACK only if address matched
              st            <= ACK;
            end
          end

          // ------------------------------------------------------------------
          // DATA (write to slave): shift data in on SCL rising edges
          // When the 8th bit completes (falling edge after last rising), arm ACK
          // ------------------------------------------------------------------
          DATA: begin
            if (scl_rise && bit_idx != 0) begin
              shreg   <= {shreg[6:0], sda_qq};
              bit_idx <= bit_idx - 4'd1;
            end else if (scl_rise && bit_idx == 0) begin
              // captured last data bit at this rising edge
            end

            if (bit_idx == 0 && scl_fall) begin
              // start ACK bit on this LOW period
              ack_active    <= 1'b1;
              ack_drive_low <= (should_nack_this_data(data_byte_count)) ? 1'b0 : 1'b1;
              st            <= ACK;
            end
          end
          
          READ: begin
            sda_drv_low <= ~tx_reg[7];
            if (scl_fall) begin
                tx_reg <= {tx_reg [6:0], 1'b0};
                if ( bit_idx != 0)  
                    bit_idx <= bit_idx - 1;
            end 
            
            if (bit_idx == 0)             
                st      <= READ_ACK;  
          end

          READ_ACK: begin
            sda_drv_low <= 1'b0;
            if(scl_rise) begin
                if (sda_qq == 1'b0) begin
                    rd_idx  <= rd_idx + 1;
                    tx_reg  <= read_mem[rd_idx+1];
                    bit_idx <= 4'd8;
                end else 
                    st      <= IDLE;
             end
             
             if(scl_fall)
                st      <= READ;
          end

          // ------------------------------------------------------------------
          // ACK bit handling:
          // - We entered ACK with ack_active=1 and ack_drive_low=decision
          // - Hold SDA low across SCL high if ack_drive_low==1
          // - On the falling edge after SCL high, release SDA, clear ack_active,
          //   and branch to next state.
          // ------------------------------------------------------------------
          ACK: begin
            if (scl_fall) begin
              // End of the ACK clock -> release SDA and move on
              ack_active    <= 1'b0;
              sda_drv_low   <= 1'b0;

              if (addr_byte) begin
                addr_byte <= 1'b0;
                bit_idx   <= 4'd8;
                if (ack_drive_low) begin                
                  // Address matched -> branch by R/W
                  if (mode_read==1'b0) begin
                    st    <= DATA;     
                    shreg <= '0;
                  end else begin
                    tx_reg  <= read_mem[rd_idx]; 
                    st    <= READ;    
                  end
                end else begin
                  // Address NACKed: ignore until next START/STOP
                  st <= IDLE;
                end
                
              end else begin
                // Data ACK just finished
                if (ack_drive_low==1'b0) begin
                  // We intentionally NACKed
                  st <= IDLE;
                end else begin
                  // ACKed data
                  data_byte_count <= data_byte_count + 1;
                  bit_idx         <= 4'd8;
                  st              <= DATA;
                end
              end
              
            end
            
          end

          default:  ;
        endcase
      end
    end
  end
endmodule

`default_nettype wire
