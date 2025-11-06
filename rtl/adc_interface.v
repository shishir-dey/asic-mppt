`timescale 1ns / 1ps

//==============================================================================
// ADC Interface and Signal Conditioning
//==============================================================================
// This module handles the 12-bit ADC inputs and converts them to fixed-point
// format for use by the MPPT algorithm. Includes scaling, filtering, and
// calibration capabilities.
//
// ADC Inputs (12-bit):
//   - Battery voltage (0-60V range)
//   - Battery current (0-20A range)
//   - Solar voltage (0-60V range)
//   - Solar current (0-20A range)
//   - Temperature 1 (0-150°C range)
//   - Temperature 2 (0-150°C range)
//==============================================================================

module adc_interface #(
    parameter ADC_BITS = 12,
    parameter DATA_WIDTH = 32,  // Q16.16 fixed-point
    parameter FILTER_DEPTH = 8  // Moving average filter depth
)(
    input wire clk,
    input wire rst_n,
    
    // ADC inputs (12-bit)
    input wire [ADC_BITS-1:0] battery_voltage_adc,
    input wire [ADC_BITS-1:0] battery_current_adc,
    input wire [ADC_BITS-1:0] solar_voltage_adc,
    input wire [ADC_BITS-1:0] solar_current_adc,
    input wire [ADC_BITS-1:0] temperature_1_adc,
    input wire [ADC_BITS-1:0] temperature_2_adc,
    
    // Calibration parameters (Q16.16 format)
    input wire signed [DATA_WIDTH-1:0] batt_v_scale,
    input wire signed [DATA_WIDTH-1:0] batt_v_offset,
    input wire signed [DATA_WIDTH-1:0] batt_i_scale,
    input wire signed [DATA_WIDTH-1:0] batt_i_offset,
    input wire signed [DATA_WIDTH-1:0] solar_v_scale,
    input wire signed [DATA_WIDTH-1:0] solar_v_offset,
    input wire signed [DATA_WIDTH-1:0] solar_i_scale,
    input wire signed [DATA_WIDTH-1:0] solar_i_offset,
    
    // Filtered and scaled outputs (Q16.16 format)
    output reg signed [DATA_WIDTH-1:0] battery_voltage,
    output reg signed [DATA_WIDTH-1:0] battery_current,
    output reg signed [DATA_WIDTH-1:0] solar_voltage,
    output reg signed [DATA_WIDTH-1:0] solar_current,
    output reg signed [DATA_WIDTH-1:0] temperature_1,
    output reg signed [DATA_WIDTH-1:0] temperature_2,
    
    // Status
    output reg data_valid
);

    // Moving average filter for each channel
    reg [ADC_BITS-1:0] batt_v_buffer [0:FILTER_DEPTH-1];
    reg [ADC_BITS-1:0] batt_i_buffer [0:FILTER_DEPTH-1];
    reg [ADC_BITS-1:0] solar_v_buffer [0:FILTER_DEPTH-1];
    reg [ADC_BITS-1:0] solar_i_buffer [0:FILTER_DEPTH-1];
    reg [ADC_BITS-1:0] temp1_buffer [0:FILTER_DEPTH-1];
    reg [ADC_BITS-1:0] temp2_buffer [0:FILTER_DEPTH-1];
    
    reg [ADC_BITS+3-1:0] batt_v_sum;
    reg [ADC_BITS+3-1:0] batt_i_sum;
    reg [ADC_BITS+3-1:0] solar_v_sum;
    reg [ADC_BITS+3-1:0] solar_i_sum;
    reg [ADC_BITS+3-1:0] temp1_sum;
    reg [ADC_BITS+3-1:0] temp2_sum;
    
    reg [ADC_BITS-1:0] batt_v_avg;
    reg [ADC_BITS-1:0] batt_i_avg;
    reg [ADC_BITS-1:0] solar_v_avg;
    reg [ADC_BITS-1:0] solar_i_avg;
    reg [ADC_BITS-1:0] temp1_avg;
    reg [ADC_BITS-1:0] temp2_avg;
    
    integer i;
    reg [2:0] filter_index;
    reg [3:0] sample_count;
    
    // Default scaling factors (can be overridden via calibration)
    // Battery voltage: 0-60V -> 0-4095 ADC counts
    // Scale = 60V / 4095 = 0.01465 V/count = 0x0000_03C0 in Q16.16
    localparam signed [DATA_WIDTH-1:0] DEFAULT_BATT_V_SCALE = 32'h0000_03C0;
    localparam signed [DATA_WIDTH-1:0] DEFAULT_BATT_I_SCALE = 32'h0000_0140; // 20A / 4095
    localparam signed [DATA_WIDTH-1:0] DEFAULT_SOLAR_V_SCALE = 32'h0000_03C0;
    localparam signed [DATA_WIDTH-1:0] DEFAULT_SOLAR_I_SCALE = 32'h0000_0140;
    localparam signed [DATA_WIDTH-1:0] DEFAULT_TEMP_SCALE = 32'h0000_0260; // 150°C / 4095
    
    // State machine for filtering and conversion
    localparam IDLE = 3'd0;
    localparam SAMPLE = 3'd1;
    localparam FILTER = 3'd2;
    localparam SCALE = 3'd3;
    localparam DONE = 3'd4;
    
    reg [2:0] state;
    reg [2:0] scale_channel;
    
    // Multiplication for scaling
    reg signed [DATA_WIDTH-1:0] scale_operand;
    reg signed [DATA_WIDTH-1:0] scale_factor;
    reg signed [DATA_WIDTH-1:0] scale_offset;
    wire signed [2*DATA_WIDTH-1:0] scale_result;
    
    assign scale_result = scale_operand * scale_factor;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            filter_index <= 0;
            sample_count <= 0;
            data_valid <= 0;
            scale_channel <= 0;
            
            // Initialize filter buffers
            for (i = 0; i < FILTER_DEPTH; i = i + 1) begin
                batt_v_buffer[i] <= 0;
                batt_i_buffer[i] <= 0;
                solar_v_buffer[i] <= 0;
                solar_i_buffer[i] <= 0;
                temp1_buffer[i] <= 0;
                temp2_buffer[i] <= 0;
            end
            
            batt_v_sum <= 0;
            batt_i_sum <= 0;
            solar_v_sum <= 0;
            solar_i_sum <= 0;
            temp1_sum <= 0;
            temp2_sum <= 0;
            
            battery_voltage <= 0;
            battery_current <= 0;
            solar_voltage <= 0;
            solar_current <= 0;
            temperature_1 <= 0;
            temperature_2 <= 0;
        end else begin
            case (state)
                IDLE: begin
                    data_valid <= 0;
                    state <= SAMPLE;
                end
                
                SAMPLE: begin
                    // Update filter buffers with new samples
                    batt_v_buffer[filter_index] <= battery_voltage_adc;
                    batt_i_buffer[filter_index] <= battery_current_adc;
                    solar_v_buffer[filter_index] <= solar_voltage_adc;
                    solar_i_buffer[filter_index] <= solar_current_adc;
                    temp1_buffer[filter_index] <= temperature_1_adc;
                    temp2_buffer[filter_index] <= temperature_2_adc;
                    
                    filter_index <= filter_index + 1;
                    if (sample_count < FILTER_DEPTH)
                        sample_count <= sample_count + 1;
                    
                    state <= FILTER;
                end
                
                FILTER: begin
                    // Calculate moving average
                    batt_v_sum = 0;
                    batt_i_sum = 0;
                    solar_v_sum = 0;
                    solar_i_sum = 0;
                    temp1_sum = 0;
                    temp2_sum = 0;
                    
                    for (i = 0; i < FILTER_DEPTH; i = i + 1) begin
                        batt_v_sum = batt_v_sum + batt_v_buffer[i];
                        batt_i_sum = batt_i_sum + batt_i_buffer[i];
                        solar_v_sum = solar_v_sum + solar_v_buffer[i];
                        solar_i_sum = solar_i_sum + solar_i_buffer[i];
                        temp1_sum = temp1_sum + temp1_buffer[i];
                        temp2_sum = temp2_sum + temp2_buffer[i];
                    end
                    
                    // Divide by FILTER_DEPTH (shift right by 3 for depth of 8)
                    batt_v_avg <= batt_v_sum >> 3;
                    batt_i_avg <= batt_i_sum >> 3;
                    solar_v_avg <= solar_v_sum >> 3;
                    solar_i_avg <= solar_i_sum >> 3;
                    temp1_avg <= temp1_sum >> 3;
                    temp2_avg <= temp2_sum >> 3;
                    
                    scale_channel <= 0;
                    state <= SCALE;
                end
                
                SCALE: begin
                    // Scale and offset each channel
                    case (scale_channel)
                        3'd0: begin
                            scale_operand <= {20'h00000, batt_v_avg};
                            scale_factor <= (batt_v_scale != 0) ? batt_v_scale : DEFAULT_BATT_V_SCALE;
                            scale_offset <= batt_v_offset;
                            scale_channel <= scale_channel + 1;
                        end
                        3'd1: begin
                            battery_voltage <= scale_result[DATA_WIDTH+11:12] + scale_offset;
                            scale_operand <= {20'h00000, batt_i_avg};
                            scale_factor <= (batt_i_scale != 0) ? batt_i_scale : DEFAULT_BATT_I_SCALE;
                            scale_offset <= batt_i_offset;
                            scale_channel <= scale_channel + 1;
                        end
                        3'd2: begin
                            battery_current <= scale_result[DATA_WIDTH+11:12] + scale_offset;
                            scale_operand <= {20'h00000, solar_v_avg};
                            scale_factor <= (solar_v_scale != 0) ? solar_v_scale : DEFAULT_SOLAR_V_SCALE;
                            scale_offset <= solar_v_offset;
                            scale_channel <= scale_channel + 1;
                        end
                        3'd3: begin
                            solar_voltage <= scale_result[DATA_WIDTH+11:12] + scale_offset;
                            scale_operand <= {20'h00000, solar_i_avg};
                            scale_factor <= (solar_i_scale != 0) ? solar_i_scale : DEFAULT_SOLAR_I_SCALE;
                            scale_offset <= solar_i_offset;
                            scale_channel <= scale_channel + 1;
                        end
                        3'd4: begin
                            solar_current <= scale_result[DATA_WIDTH+11:12] + scale_offset;
                            scale_operand <= {20'h00000, temp1_avg};
                            scale_factor <= DEFAULT_TEMP_SCALE;
                            scale_offset <= 0;
                            scale_channel <= scale_channel + 1;
                        end
                        3'd5: begin
                            temperature_1 <= scale_result[DATA_WIDTH+11:12];
                            scale_operand <= {20'h00000, temp2_avg};
                            scale_factor <= DEFAULT_TEMP_SCALE;
                            scale_offset <= 0;
                            scale_channel <= scale_channel + 1;
                        end
                        3'd6: begin
                            temperature_2 <= scale_result[DATA_WIDTH+11:12];
                            state <= DONE;
                        end
                        default: state <= DONE;
                    endcase
                end
                
                DONE: begin
                    data_valid <= 1;
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule