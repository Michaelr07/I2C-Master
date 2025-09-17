# I²C Master (SystemVerilog)

Small, self-contained I²C master with a simple slave BFM and a self-checking testbench.

## Features
- 7-bit addressing; **write**, **read**, and **combined** write > repeated-START > read
- Streaming interfaces  
  - **Write**: `wr_data[7:0]`, `wr_valid / wr_ready`  
  - **Read**: `rd_data[7:0]`, `rd_valid / rd_ready`
- Byte counts latched at `start` via `wr_len` / `rd_len`
- Open-drain pins (`sda_io`, `scl_io`), pull-ups modeled in TB with `tri1`
- Optional master-side clock stretching on read back-pressure

## Repo layout
 rtl/
    i2c_master.sv
 tb/
    i2c_slave_ack_bfm.sv
    i2c_master_tb.sv
  docs/
    sim_pictures

## Module parameters (master)

| Parameter     | Default       | Description                                  |
|---------------|---------------|----------------------------------------------|
| `SYS_CLK`     | `100_000_000` | System clock (Hz)                            |
| `I2C_SPEED`   | `100_000`     | Target I²C SCL frequency (Hz)                |
| `STRETCH_EN`  | `1`           | Allow master to stretch SCL on read back-pressure |

## Ports (master)

- **Command**: `start` (1-cycle pulse), `addr7[6:0]`, `wr_len[7:0]`, `rd_len[7:0]`
- **Status**: `busy`, `done` (1-cycle), `nack_addr`, `nack_data`, `timeout`
- **Write stream**: `wr_data[7:0]`, `wr_valid`, `wr_ready`
- **Read stream**: `rd_data[7:0]`, `rd_valid`, `rd_ready`
- **Pins**: `sda_io`, `scl_io` (open-drain: drive low or Z)

### Handshake rules
- **Write**: present `wr_data` with `wr_valid=1`; byte is accepted on `wr_ready=1`.
- **Read**: hold `rd_ready=1` to accept; sample `rd_data` on a rising edge of `rd_valid`.
- `wr_len`/`rd_len` are captured on the `start` pulse (don’t change mid-transaction).

## Protocol behavior
- **Write**: `START > addr(W) > data… > STOP`
- **Read**: `START > addr(R) → data… > master NACK last byte > STOP`
- **Combined**:  
  `START > addr(W) > data… > repeated START > addr(R) > data… > STOP`  
  (Address is sent twice; write first, then read.)
- ACK sampled on **SCL rising edge**. SDA is only changed while **SCL low** (except holding/releasing ACK across the 9th clock per spec).
- **Read back-pressure** (if `STRETCH_EN=1`): when `rd_valid=1` and `rd_ready=0`, the master holds SCL low (stretches) until the consumer is ready.

## Testbench
Self-checking tests cover:
- Write (good/bad address)
- Read N bytes (good/bad address)
- Combined write and read with repeated START

## The slave BFM:
- ACKs addresses that match `SLAVE_ADDR7`
- ACKs data writes by default
- On reads, returns a simple sequence `A0, A1, A2, A3, …`

## Limitations
- 7-bit addressing only (no 10-bit)
- No multi-master or arbitration
- BFM is minimal (no real register map)

    
