`timescale 1ns / 1ps

//==============================================================================
// I2C Slave Interface for MPPT Configuration and Monitoring
//==============================================================================
// This module implements an I2C slave interface for reading/writing MPPT
// configuration parameters and monitoring system status.
//
// Register Map:
//   0x00: Control Register (R/W)
//   0x01: Status Register (R)
//   0x02-0x03: Step Size (R/W) - 16-bit
//   0x04-0x05: V_max (R/W) - 16-bit
//   0x06-0x07: V_min (R/W) - 16-bit
//   0x08-0x09: V_ref (R) - 16-bit
//   0x0A-0x0B: PV Voltage (R) - 16-bit
//   0x0C-0x0D: PV Current (R) - 16-bit
//   0x0E-0x0F: Battery Voltage (R) - 16-bit
//   0x10-0x11: Battery Current (R) - 16-bit
//   0x12-0x13: Temperature 1 (R) - 16-bit
//   0x14-0x15: Temperature 2 (R) - 16-bit
//==============================================================================

module i2c_slave #(
    parameter SLAVE_ADDR = 7'h50  // Default I2C address
)(
    input wire clk,
    input wire rst_n,
    
    // I2C interface
    inout wire sda,
    input wire scl,
    
    // Register interface
    output reg [7:0] reg_addr,
    output reg [7:0] reg_wdata,
    output reg reg_write,
    input wire [7:0] reg_rdata,
    output reg reg_read
);

    // I2C state machine
    localparam IDLE = 4'd0;
    localparam START = 4'd1;
    localparam ADDR = 4'd2;
    localparam ADDR_ACK = 4'd3;
    localparam DATA_RX = 4'd4;
    localparam DATA_RX_ACK = 4'd5;
    localparam DATA_TX = 4'd6;
    localparam DATA_TX_ACK = 4'd7;
    localparam STOP = 4'd8;
    
    reg [3:0] state;
    reg [3:0] bit_count;
    reg [7:0] shift_reg;
    reg [6:0] addr_reg;
    reg rw_bit;
    reg sda_out;
    reg sda_oe;  // Output enable for SDA
    
    // Edge detection for SCL
    reg scl_d1, scl_d2;
    wire scl_posedge = scl_d1 && !scl_d2;
    wire scl_negedge = !scl_d1 && scl_d2;
    
    // Edge detection for SDA (for START/STOP conditions)
    reg sda_d1, sda_d2;
    wire sda_negedge = !sda_d1 && sda_d2;
    wire sda_posedge = sda_d1 && !sda_d2;
    
    // START condition: SDA falling edge while SCL is high
    wire start_cond = sda_negedge && scl_d2;
    
    // STOP condition: SDA rising edge while SCL is high
    wire stop_cond = sda_posedge && scl_d2;
    
    // Tri-state buffer for SDA
    assign sda = sda_oe ? sda_out : 1'bz;
    
    // Synchronize SCL and SDA
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_d1 <= 1;
            scl_d2 <= 1;
            sda_d1 <= 1;
            sda_d2 <= 1;
        end else begin
            scl_d1 <= scl;
            scl_d2 <= scl_d1;
            sda_d1 <= sda;
            sda_d2 <= sda_d1;
        end
    end
    
    // Main I2C state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_count <= 0;
            shift_reg <= 0;
            addr_reg <= 0;
            rw_bit <= 0;
            sda_out <= 1;
            sda_oe <= 0;
            reg_addr <= 0;
            reg_wdata <= 0;
            reg_write <= 0;
            reg_read <= 0;
        end else begin
            // Default values
            reg_write <= 0;
            reg_read <= 0;
            
            case (state)
                IDLE: begin
                    sda_oe <= 0;
                    bit_count <= 0;
                    if (start_cond) begin
                        state <= START;
                    end
                end
                
                START: begin
                    if (scl_posedge) begin
                        state <= ADDR;
                        bit_count <= 7;
                    end
                end
                
                ADDR: begin
                    if (scl_posedge) begin
                        shift_reg <= {shift_reg[6:0], sda_d2};
                        if (bit_count == 0) begin
                            addr_reg <= shift_reg[7:1];
                            rw_bit <= shift_reg[0];
                            state <= ADDR_ACK;
                        end else begin
                            bit_count <= bit_count - 1;
                        end
                    end
                end
                
                ADDR_ACK: begin
                    if (addr_reg == SLAVE_ADDR) begin
                        // Address matches, send ACK
                        if (scl_negedge) begin
                            sda_out <= 0;
                            sda_oe <= 1;
                        end
                        if (scl_posedge) begin
                            bit_count <= 7;
                            if (rw_bit) begin
                                state <= DATA_TX;
                                reg_read <= 1;
                            end else begin
                                state <= DATA_RX;
                            end
                        end
                    end else begin
                        // Address doesn't match, return to IDLE
                        state <= IDLE;
                    end
                end
                
                DATA_RX: begin
                    sda_oe <= 0;
                    if (scl_posedge) begin
                        shift_reg <= {shift_reg[6:0], sda_d2};
                        if (bit_count == 0) begin
                            state <= DATA_RX_ACK;
                            if (reg_addr == 0) begin
                                // First byte is register address
                                reg_addr <= shift_reg;
                            end else begin
                                // Subsequent bytes are data
                                reg_wdata <= shift_reg;
                                reg_write <= 1;
                                reg_addr <= reg_addr + 1;
                            end
                        end else begin
                            bit_count <= bit_count - 1;
                        end
                    end
                end
                
                DATA_RX_ACK: begin
                    if (scl_negedge) begin
                        sda_out <= 0;
                        sda_oe <= 1;
                    end
                    if (scl_posedge) begin
                        bit_count <= 7;
                        state <= DATA_RX;
                    end
                    if (stop_cond) begin
                        state <= IDLE;
                        sda_oe <= 0;
                    end
                end
                
                DATA_TX: begin
                    if (scl_negedge) begin
                        sda_out <= shift_reg[7];
                        sda_oe <= 1;
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        if (bit_count == 7) begin
                            shift_reg <= reg_rdata;
                        end
                    end
                    if (scl_posedge) begin
                        if (bit_count == 0) begin
                            state <= DATA_TX_ACK;
                        end else begin
                            bit_count <= bit_count - 1;
                        end
                    end
                end
                
                DATA_TX_ACK: begin
                    if (scl_negedge) begin
                        sda_oe <= 0;
                    end
                    if (scl_posedge) begin
                        if (sda_d2) begin
                            // NACK received, stop transmission
                            state <= IDLE;
                        end else begin
                            // ACK received, continue
                            bit_count <= 7;
                            reg_addr <= reg_addr + 1;
                            reg_read <= 1;
                            state <= DATA_TX;
                        end
                    end
                    if (stop_cond) begin
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
            
            // Handle STOP condition in any state
            if (stop_cond && state != IDLE) begin
                state <= IDLE;
                sda_oe <= 0;
            end
        end
    end

endmodule