`timescale 1ns / 1ps
`default_nettype none

  module i2c_master #(
    parameter int SYS_CLK       = 100_000_000,  //Hz
    parameter int I2C_SPEED     = 100_000,      //Hz
    parameter bit STRETCH_EN    = 1             //Clock stretching
) (
    input wire logic            clk,
    input wire logic            rst_n,
    
    // Command
    input  wire logic           start,          // pulse high to launch a transaction
    input  wire logic [6:0]     addr7,          // 7-bit slave address
    input  wire logic [7:0]     wr_len,         // number of bytes to write
    input  wire logic [7:0]     rd_len,         // number of bytes to r7       
                     
    output logic                busy,           // high during transaction
    output logic                done,           // 1-cycle pulse when complete
    output logic                nack_addr,      // address phase NACK
    output logic                nack_data,      // data phase NACK
    output logic                timeout,        // SCL held low too long (stretch fail)
                                       
    input wire logic [7:0]      wr_data,        // write data stream
    input wire logic            wr_valid,
    output logic                wr_ready,
    
    output logic [7:0]          rd_data,        // read data stream
    output logic                rd_valid,
    input wire logic            rd_ready,
    
    inout tri                   sda_io,            // I2C pins
    inout tri                   scl_io
);

    logic sda_drv_low, scl_drv_low;
    logic sda_meta, sda_in;
    logic scl_meta, scl_in;
    
    assign sda_io = sda_drv_low ? 1'b0 : 1'bz;
    assign scl_io = scl_drv_low ? 1'b0 : 1'bz;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)begin
            sda_meta <= 1'b1;
            scl_meta <= 1'b1;
            sda_in   <= 1'b1;
            scl_in   <= 1'b1;
        end else begin
            sda_meta <= sda_io;
            sda_in   <= sda_meta;
            scl_meta <= scl_io;
            scl_in   <= scl_meta;
        end
    end
    
    //Divider for half period of scl
    localparam int unsigned DIV = (SYS_CLK + (2*I2C_SPEED - 1))/(2*I2C_SPEED);
    logic tick;
    
    n_counter #(.DIV(DIV)) clk_pulse (.clk(clk), .rst_n(rst_n), .en(1'b1), .done(tick));
    
    wire scl_rise = scl_meta & ~scl_in;
    wire scl_fall = ~scl_meta & scl_in;
    
    //START and STOP FSM
    logic [3:0] bit_counter;
    logic [7:0] shift_reg;
    
    //Control logic for write/read
    logic [7:0] tx_rem, rx_rem;   // remaining write bytes
    logic do_write, do_read, do_both;
    logic addr_byte, stop_bit;
    logic wr_accept_q, stop_now;
    
    typedef enum logic [3:0] {
        IDLE, 
        START_HI, START_FALL, START_HOLD,
        ADDR_LOAD,
        BYTE_LOAD, BIT_LOW, BIT_HIGH, 
        ACK_LOW, ACK_HIGH, ACK_CHECK,
        STOP_PREP, STOP_RISE, DONE
    } 
        state_type;
        
    state_type next, state;
    
        //start edge detect (1 cycle pulse)
    logic start_q, start_qq, start_pulse, start_ok;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) begin 
            start_q     <= 1'b0; 
            start_qq    <= 1'b0;
        end else begin       
            start_q     <= start;
            start_qq    <= start_q;
        end
        
//    always_ff @(posedge clk or negedge rst_n) 
//        if (!rst_n) wr_accept_q <= 1'b0;
//        else        wr_accept_q <= (state == BYTE_LOAD) && wr_valid;
    
    wire wr_accept = (state == BYTE_LOAD) && wr_valid;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) wr_ready    <= 1'b0;
        else        wr_ready    <= (state == BYTE_LOAD);
    
    always_ff @(posedge clk or negedge rst_n) 
        if (!rst_n)     shift_reg <= '0;
        else if (addr_byte);        // maybe delete
        else if (wr_accept) shift_reg <= wr_data;
        
   // stable, second-stage SCL edges inside clk domain
    logic scl_in_d;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) scl_in_d <= 1'b0;
        else        scl_in_d <= scl_in;

    wire scl_rise_sync =  scl_in & ~scl_in_d;   // use these, not tick
    wire scl_fall_sync = ~scl_in &  scl_in_d;

    assign start_pulse  = ~start_qq & start_q;
    assign start_ok     = start_pulse && ((wr_len > 0) || (rd_len > 0));
    assign stop_bit     = nack_addr || nack_data || (!addr_byte && do_write && (tx_rem == 0));


    //state register
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else        state <= next;
    
    // state logic
    always_comb begin
        next = state;
        case (state)
            IDLE      : if (start_ok)           next = START_HI;
            
            START_HI  : if (scl_in && sda_in)   next = START_FALL;
            
            START_FALL:                         next = START_HOLD;
            
            START_HOLD: if (tick)               next = ADDR_LOAD;
            
            ADDR_LOAD :                         next = BIT_LOW;
            
            BYTE_LOAD : if (wr_accept)          next = BIT_LOW; //fix maybe
            
            BIT_LOW:    if (tick)               next = BIT_HIGH;
            
            BIT_HIGH  : if (tick)               next = (bit_counter == 8)? ACK_LOW : BIT_LOW; // adjust for multi byte 
            
            ACK_LOW  :  if (tick)               next = ACK_HIGH;
            
            ACK_HIGH :  if (tick)               next = ACK_CHECK;
                 
            ACK_CHECK:                          next = (stop_bit)? STOP_PREP : BYTE_LOAD;
            
            STOP_PREP:  if (tick)               next = STOP_RISE;
            
            STOP_RISE:  if (tick && scl_in)     next = DONE;      
             
            DONE:                               next = IDLE;
            default:                            next = IDLE;
        endcase 
    end
 
    //OUTPUT REGISTER
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            nack_addr   <= 1'b0;
            nack_data   <= 1'b0;
            timeout     <= 1'b0; 
            rd_data     <= '0;
            rd_valid    <= 1'b0;
            sda_drv_low <= 1'b0;
            scl_drv_low <= 1'b0;
            busy        <= 1'b0;
            done        <= 1'b0;
            shift_reg   <= '0;
            bit_counter <= '0;
            do_write    <= 1'b0;
            do_read     <= 1'b0;
            do_both     <= 1'b0;
            addr_byte   <= 1'b0;
            tx_rem      <= 1'b0;
        end
        else begin
            case (next)
                IDLE                    :   begin
                                                sda_drv_low <= 1'b0;
                                                scl_drv_low <= 1'b0;
                                                nack_addr   <= 1'b0;
                                                nack_data   <= 1'b0;
                                                timeout     <= 1'b0;
                                                rd_data     <= '0;
                                                rd_valid    <= 1'b0;
                                                done        <= 1'b0;
                                                busy        <= 1'b0;
                                                do_write    <= 1'b0;
                                                do_read     <= 1'b0;
                                                do_both     <= 1'b0;
                                                addr_byte   <= 1'b0;
                                                tx_rem      <= '0;
                                            end
                START_HI                :   begin
                                                sda_drv_low <= 1'b0;
                                                scl_drv_low <= 1'b0;
                                                busy        <= 1'b1;
                                                if (wr_len > 0 && rd_len > 0) begin
                                                   do_both  <= 1'b1;
                                                   tx_rem   <= wr_len;
                                                   rx_rem   <= rd_len;
                                                end else if (wr_len > 0) begin
                                                   do_write <= 1'b1;
                                                   tx_rem   <= wr_len;
                                                end else if (rd_len > 0) begin
                                                   do_read  <= 1'b1;
                                                   rx_rem   <= rd_len;
                                                end                                                       
                                            end
                START_FALL, START_HOLD  :   begin
                                                sda_drv_low <= 1'b1;
                                                scl_drv_low <= 1'b0;
                                            end
                ADDR_LOAD               :   begin                                               
                                                sda_drv_low <= 1'b1;                          
                                                scl_drv_low <= 1'b1;
                                                addr_byte   <= 1'b1;
                                                bit_counter <= '0;
                                                shift_reg   <= (do_write || do_both)? {addr7,1'b0} : {addr7,1'b1};
                                            end    
                                            
                BYTE_LOAD               :   begin                                       //fix                       
                                                sda_drv_low <= 1'b1;                             
                                                scl_drv_low <= 1'b1;
                                                bit_counter <= '0;
                                            end
                BIT_LOW                 :   begin
                                                scl_drv_low <= 1'b1;
                                                sda_drv_low <= (shift_reg[7])? 1'b0 : 1'b1;
                                            end
                BIT_HIGH                :   begin
                                                scl_drv_low <= 1'b0;
                                                if (scl_rise_sync) begin
                                                    shift_reg   <= {shift_reg[6:0],1'b0};
                                                    bit_counter <= bit_counter + 1;
                                               end
                                            end
                ACK_LOW                 :   begin
                                                scl_drv_low <= 1'b1;
                                                sda_drv_low <= 1'b0;    // maybe the  prob
                                                //shift_reg   <= '0;
                                            end
                ACK_HIGH                :   begin
                                                scl_drv_low <= 1'b0;
                                                if (scl_rise) begin
                                                    if (addr_byte) begin
                                                       nack_addr <= sda_in;
                                                      // addr_byte <= 1'b0;
                                                    end else if (do_write) begin
                                                       nack_data <= sda_in;
                                                    end  
                                                end
                                            end
                ACK_CHECK               :   begin
                                            // byte counter decrement
                                            //if good then continue if theres still bytes left
                                            if (!nack_data && do_write && !addr_byte && tx_rem != 0)
                                                tx_rem      <= tx_rem - 1;
                                            else
                                                addr_byte <= 1'b0;
                                                //wr_ready    <= 1'b1;
                                            end             
                STOP_PREP               :   begin
                                                sda_drv_low <= 1'b1;
                                                scl_drv_low <= 1'b0;    // release SCL used to be 1
                                            end 
                STOP_RISE               :   begin
                                                sda_drv_low <= (scl_in)? 1'b0 : 1'b1;
                                                scl_drv_low <= 1'b0;
                                            end
               DONE                     :       done        <= 1'b1;
               default: ;                   //to apply defaults
            endcase
       end
    end
  
endmodule
