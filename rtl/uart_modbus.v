`timescale 1ns / 1ps

//==============================================================================
// UART/Modbus RTU Interface
//==============================================================================
// This module implements a UART interface with Modbus RTU protocol support
// for remote monitoring and control of the MPPT system.
//
// UART Configuration: 9600 baud, 8N1 (8 data bits, no parity, 1 stop bit)
// Modbus RTU: Standard function codes for reading/writing registers
//==============================================================================

module uart_modbus #(
    parameter CLK_FREQ = 50_000_000,  // 50 MHz system clock
    parameter BAUD_RATE = 9600,
    parameter MODBUS_ADDR = 8'h01     // Modbus device address
)(
    input wire clk,
    input wire rst_n,
    
    // UART interface
    input wire uart_rx,
    output reg uart_tx,
    
    // Register interface
    output reg [7:0] reg_addr,
    output reg [7:0] reg_wdata,
    output reg reg_write,
    input wire [7:0] reg_rdata,
    output reg reg_read,
    
    // Status
    output reg frame_error,
    output reg crc_error
);

    // Baud rate generator
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;
    reg [$clog2(BAUD_DIV)-1:0] baud_counter;
    reg baud_tick;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_counter <= 0;
            baud_tick <= 0;
        end else begin
            if (baud_counter == BAUD_DIV - 1) begin
                baud_counter <= 0;
                baud_tick <= 1;
            end else begin
                baud_counter <= baud_counter + 1;
                baud_tick <= 0;
            end
        end
    end
    
    // UART RX state machine
    localparam RX_IDLE = 3'd0;
    localparam RX_START = 3'd1;
    localparam RX_DATA = 3'd2;
    localparam RX_STOP = 3'd3;
    
    reg [2:0] rx_state;
    reg [3:0] rx_bit_count;
    reg [7:0] rx_data;
    reg rx_data_valid;
    reg [3:0] rx_sample_count;
    
    // Synchronize UART RX
    reg uart_rx_d1, uart_rx_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_rx_d1 <= 1;
            uart_rx_d2 <= 1;
        end else begin
            uart_rx_d1 <= uart_rx;
            uart_rx_d2 <= uart_rx_d1;
        end
    end
    
    // UART RX logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            rx_bit_count <= 0;
            rx_data <= 0;
            rx_data_valid <= 0;
            rx_sample_count <= 0;
            frame_error <= 0;
        end else begin
            rx_data_valid <= 0;
            
            case (rx_state)
                RX_IDLE: begin
                    if (!uart_rx_d2) begin
                        rx_state <= RX_START;
                        rx_sample_count <= 0;
                    end
                end
                
                RX_START: begin
                    if (baud_tick) begin
                        if (rx_sample_count == 7) begin
                            if (!uart_rx_d2) begin
                                rx_state <= RX_DATA;
                                rx_bit_count <= 0;
                                rx_sample_count <= 0;
                            end else begin
                                rx_state <= RX_IDLE;
                                frame_error <= 1;
                            end
                        end else begin
                            rx_sample_count <= rx_sample_count + 1;
                        end
                    end
                end
                
                RX_DATA: begin
                    if (baud_tick) begin
                        if (rx_sample_count == 15) begin
                            rx_data <= {uart_rx_d2, rx_data[7:1]};
                            rx_sample_count <= 0;
                            if (rx_bit_count == 7) begin
                                rx_state <= RX_STOP;
                            end else begin
                                rx_bit_count <= rx_bit_count + 1;
                            end
                        end else begin
                            rx_sample_count <= rx_sample_count + 1;
                        end
                    end
                end
                
                RX_STOP: begin
                    if (baud_tick) begin
                        if (rx_sample_count == 15) begin
                            if (uart_rx_d2) begin
                                rx_data_valid <= 1;
                                frame_error <= 0;
                            end else begin
                                frame_error <= 1;
                            end
                            rx_state <= RX_IDLE;
                        end else begin
                            rx_sample_count <= rx_sample_count + 1;
                        end
                    end
                end
                
                default: rx_state <= RX_IDLE;
            endcase
        end
    end
    
    // UART TX state machine
    localparam TX_IDLE = 3'd0;
    localparam TX_START = 3'd1;
    localparam TX_DATA = 3'd2;
    localparam TX_STOP = 3'd3;
    
    reg [2:0] tx_state;
    reg [3:0] tx_bit_count;
    reg [7:0] tx_data;
    reg tx_start;
    reg tx_busy;
    reg [3:0] tx_sample_count;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_bit_count <= 0;
            uart_tx <= 1;
            tx_busy <= 0;
            tx_sample_count <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    uart_tx <= 1;
                    tx_busy <= 0;
                    if (tx_start) begin
                        tx_state <= TX_START;
                        tx_busy <= 1;
                        tx_sample_count <= 0;
                    end
                end
                
                TX_START: begin
                    uart_tx <= 0;
                    if (baud_tick) begin
                        if (tx_sample_count == 15) begin
                            tx_state <= TX_DATA;
                            tx_bit_count <= 0;
                            tx_sample_count <= 0;
                        end else begin
                            tx_sample_count <= tx_sample_count + 1;
                        end
                    end
                end
                
                TX_DATA: begin
                    uart_tx <= tx_data[0];
                    if (baud_tick) begin
                        if (tx_sample_count == 15) begin
                            tx_data <= {1'b0, tx_data[7:1]};
                            tx_sample_count <= 0;
                            if (tx_bit_count == 7) begin
                                tx_state <= TX_STOP;
                            end else begin
                                tx_bit_count <= tx_bit_count + 1;
                            end
                        end else begin
                            tx_sample_count <= tx_sample_count + 1;
                        end
                    end
                end
                
                TX_STOP: begin
                    uart_tx <= 1;
                    if (baud_tick) begin
                        if (tx_sample_count == 15) begin
                            tx_state <= TX_IDLE;
                        end else begin
                            tx_sample_count <= tx_sample_count + 1;
                        end
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
    
    // Modbus RTU protocol handler
    localparam MB_IDLE = 4'd0;
    localparam MB_ADDR = 4'd1;
    localparam MB_FUNC = 4'd2;
    localparam MB_REG_HI = 4'd3;
    localparam MB_REG_LO = 4'd4;
    localparam MB_DATA_HI = 4'd5;
    localparam MB_DATA_LO = 4'd6;
    localparam MB_CRC_LO = 4'd7;
    localparam MB_CRC_HI = 4'd8;
    localparam MB_PROCESS = 4'd9;
    localparam MB_RESPOND = 4'd10;
    
    reg [3:0] mb_state;
    reg [7:0] mb_addr_rx;
    reg [7:0] mb_func;
    reg [15:0] mb_reg_addr;
    reg [15:0] mb_data;
    reg [15:0] mb_crc_rx;
    reg [15:0] mb_crc_calc;
    reg [3:0] mb_byte_count;
    
    // Simple CRC-16 Modbus calculation
    function [15:0] crc16_update;
        input [15:0] crc;
        input [7:0] data;
        integer i;
        reg [15:0] temp;
        begin
            temp = crc ^ {8'h00, data};
            for (i = 0; i < 8; i = i + 1) begin
                if (temp[0])
                    temp = (temp >> 1) ^ 16'hA001;
                else
                    temp = temp >> 1;
            end
            crc16_update = temp;
        end
    endfunction
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mb_state <= MB_IDLE;
            mb_addr_rx <= 0;
            mb_func <= 0;
            mb_reg_addr <= 0;
            mb_data <= 0;
            mb_crc_rx <= 0;
            mb_crc_calc <= 16'hFFFF;
            mb_byte_count <= 0;
            reg_write <= 0;
            reg_read <= 0;
            tx_start <= 0;
            crc_error <= 0;
        end else begin
            reg_write <= 0;
            reg_read <= 0;
            tx_start <= 0;
            
            if (rx_data_valid) begin
                case (mb_state)
                    MB_IDLE: begin
                        mb_addr_rx <= rx_data;
                        mb_crc_calc <= crc16_update(16'hFFFF, rx_data);
                        if (rx_data == MODBUS_ADDR) begin
                            mb_state <= MB_FUNC;
                        end
                    end
                    
                    MB_FUNC: begin
                        mb_func <= rx_data;
                        mb_crc_calc <= crc16_update(mb_crc_calc, rx_data);
                        mb_state <= MB_REG_HI;
                    end
                    
                    MB_REG_HI: begin
                        mb_reg_addr[15:8] <= rx_data;
                        mb_crc_calc <= crc16_update(mb_crc_calc, rx_data);
                        mb_state <= MB_REG_LO;
                    end
                    
                    MB_REG_LO: begin
                        mb_reg_addr[7:0] <= rx_data;
                        mb_crc_calc <= crc16_update(mb_crc_calc, rx_data);
                        mb_state <= MB_DATA_HI;
                    end
                    
                    MB_DATA_HI: begin
                        mb_data[15:8] <= rx_data;
                        mb_crc_calc <= crc16_update(mb_crc_calc, rx_data);
                        mb_state <= MB_DATA_LO;
                    end
                    
                    MB_DATA_LO: begin
                        mb_data[7:0] <= rx_data;
                        mb_crc_calc <= crc16_update(mb_crc_calc, rx_data);
                        mb_state <= MB_CRC_LO;
                    end
                    
                    MB_CRC_LO: begin
                        mb_crc_rx[7:0] <= rx_data;
                        mb_state <= MB_CRC_HI;
                    end
                    
                    MB_CRC_HI: begin
                        mb_crc_rx[15:8] <= rx_data;
                        mb_state <= MB_PROCESS;
                    end
                    
                    default: mb_state <= MB_IDLE;
                endcase
            end
            
            if (mb_state == MB_PROCESS) begin
                if (mb_crc_rx == mb_crc_calc) begin
                    crc_error <= 0;
                    // Process Modbus function
                    case (mb_func)
                        8'h03: begin // Read holding registers
                            reg_addr <= mb_reg_addr[7:0];
                            reg_read <= 1;
                        end
                        8'h06: begin // Write single register
                            reg_addr <= mb_reg_addr[7:0];
                            reg_wdata <= mb_data[7:0];
                            reg_write <= 1;
                        end
                        default: begin
                            // Unsupported function
                        end
                    endcase
                end else begin
                    crc_error <= 1;
                end
                mb_state <= MB_IDLE;
            end
        end
    end

endmodule