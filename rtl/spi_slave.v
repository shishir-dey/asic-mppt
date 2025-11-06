`timescale 1ns / 1ps

//==============================================================================
// SPI Slave Interface for Fast Data Readout
//==============================================================================
// This module implements an SPI slave interface for high-speed data readout
// of MPPT measurements and status. Supports Mode 0 (CPOL=0, CPHA=0).
//
// Protocol:
//   - First byte: Register address
//   - Subsequent bytes: Data read from that address (auto-increment)
//   - CS low activates the interface
//   - Data is clocked on rising edge of SCK
//==============================================================================

module spi_slave (
    input wire clk,
    input wire rst_n,
    
    // SPI interface
    input wire spi_sck,
    input wire spi_mosi,
    output reg spi_miso,
    input wire spi_ss,
    
    // Register interface
    output reg [7:0] reg_addr,
    input wire [7:0] reg_rdata,
    output reg reg_read
);

    // SPI state machine
    localparam IDLE = 2'd0;
    localparam ADDR = 2'd1;
    localparam DATA = 2'd2;
    
    reg [1:0] state;
    reg [2:0] bit_count;
    reg [7:0] shift_reg_rx;
    reg [7:0] shift_reg_tx;
    
    // Synchronize SPI signals
    reg spi_sck_d1, spi_sck_d2;
    reg spi_ss_d1, spi_ss_d2;
    reg spi_mosi_d1, spi_mosi_d2;
    
    wire spi_sck_posedge = spi_sck_d1 && !spi_sck_d2;
    wire spi_sck_negedge = !spi_sck_d1 && spi_sck_d2;
    wire spi_ss_active = !spi_ss_d2;
    wire spi_ss_negedge = !spi_ss_d1 && spi_ss_d2;
    
    // Synchronization
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sck_d1 <= 0;
            spi_sck_d2 <= 0;
            spi_ss_d1 <= 1;
            spi_ss_d2 <= 1;
            spi_mosi_d1 <= 0;
            spi_mosi_d2 <= 0;
        end else begin
            spi_sck_d1 <= spi_sck;
            spi_sck_d2 <= spi_sck_d1;
            spi_ss_d1 <= spi_ss;
            spi_ss_d2 <= spi_ss_d1;
            spi_mosi_d1 <= spi_mosi;
            spi_mosi_d2 <= spi_mosi_d1;
        end
    end
    
    // SPI state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_count <= 7;
            shift_reg_rx <= 0;
            shift_reg_tx <= 0;
            spi_miso <= 0;
            reg_addr <= 0;
            reg_read <= 0;
        end else begin
            reg_read <= 0;
            
            case (state)
                IDLE: begin
                    bit_count <= 7;
                    spi_miso <= 0;
                    if (spi_ss_negedge) begin
                        state <= ADDR;
                        shift_reg_rx <= 0;
                    end
                end
                
                ADDR: begin
                    if (!spi_ss_active) begin
                        state <= IDLE;
                    end else if (spi_sck_posedge) begin
                        // Shift in address on rising edge
                        shift_reg_rx <= {shift_reg_rx[6:0], spi_mosi_d2};
                        
                        if (bit_count == 0) begin
                            // Address received, prepare for data
                            reg_addr <= {shift_reg_rx[6:0], spi_mosi_d2};
                            reg_read <= 1;
                            state <= DATA;
                            bit_count <= 7;
                        end else begin
                            bit_count <= bit_count - 1;
                        end
                    end
                end
                
                DATA: begin
                    if (!spi_ss_active) begin
                        state <= IDLE;
                    end else begin
                        if (spi_sck_negedge) begin
                            // Update MISO on falling edge
                            if (bit_count == 7) begin
                                shift_reg_tx <= reg_rdata;
                                spi_miso <= reg_rdata[7];
                            end else begin
                                spi_miso <= shift_reg_tx[7];
                            end
                            shift_reg_tx <= {shift_reg_tx[6:0], 1'b0};
                        end
                        
                        if (spi_sck_posedge) begin
                            if (bit_count == 0) begin
                                // Byte complete, auto-increment address
                                reg_addr <= reg_addr + 1;
                                reg_read <= 1;
                                bit_count <= 7;
                            end else begin
                                bit_count <= bit_count - 1;
                            end
                        end
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule