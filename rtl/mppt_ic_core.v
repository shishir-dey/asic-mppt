`timescale 1ns / 1ps

//==============================================================================
// Incremental Conductance MPPT Core Algorithm
//==============================================================================
// This module implements the Incremental Conductance Maximum Power Point
// Tracking algorithm based on the principle that dP/dV = 0 at MPP.
//
// Theory:
//   dP/dV = d(VI)/dV = I + V(dI/dV) = 0 at MPP
//   Therefore: dI/dV = -I/V at MPP
//
// Algorithm:
//   - If dI/dV > -I/V: Increase voltage (left of MPP)
//   - If dI/dV < -I/V: Decrease voltage (right of MPP)
//   - If dI/dV = -I/V: At MPP (no change needed)
//==============================================================================

module mppt_ic_core #(
    parameter DATA_WIDTH = 32,      // Q16.16 fixed-point format
    parameter FRAC_BITS = 16
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    
    // Current measurements (12-bit ADC converted to fixed-point)
    input wire signed [DATA_WIDTH-1:0] pv_current,
    input wire signed [DATA_WIDTH-1:0] pv_voltage,
    
    // Configuration parameters
    input wire signed [DATA_WIDTH-1:0] step_size,
    input wire signed [DATA_WIDTH-1:0] v_max,
    input wire signed [DATA_WIDTH-1:0] v_min,
    
    // Output voltage reference
    output reg signed [DATA_WIDTH-1:0] v_ref,
    output reg valid,
    
    // Status outputs
    output reg at_mpp,
    output reg [1:0] state
);

    // State machine states
    localparam IDLE = 2'b00;
    localparam CALC = 2'b01;
    localparam UPDATE = 2'b10;
    localparam WAIT = 2'b11;
    
    // Internal registers
    reg signed [DATA_WIDTH-1:0] pv_i_old;
    reg signed [DATA_WIDTH-1:0] pv_v_old;
    reg signed [DATA_WIDTH-1:0] delta_i;
    reg signed [DATA_WIDTH-1:0] delta_v;
    reg signed [DATA_WIDTH-1:0] conductance;          // I/V
    reg signed [DATA_WIDTH-1:0] inc_conductance;      // dI/dV
    reg signed [DATA_WIDTH-1:0] neg_conductance;      // -I/V
    
    reg first_sample;
    reg [3:0] calc_step;
    
    // Math unit interface
    reg signed [DATA_WIDTH-1:0] math_op_a;
    reg signed [DATA_WIDTH-1:0] math_op_b;
    reg [2:0] math_operation;
    reg math_start;
    wire signed [DATA_WIDTH-1:0] math_result;
    wire math_done;
    wire math_overflow;
    wire [1:0] math_compare;
    
    // Instantiate fixed-point math unit
    fixed_point_math #(
        .INT_BITS(16),
        .FRAC_BITS(16),
        .TOTAL_BITS(32)
    ) math_unit (
        .clk(clk),
        .rst_n(rst_n),
        .operand_a(math_op_a),
        .operand_b(math_op_b),
        .operation(math_operation),
        .start(math_start),
        .result(math_result),
        .done(math_done),
        .overflow(math_overflow),
        .compare_result(math_compare)
    );
    
    // Threshold for detecting MPP (small value in fixed-point)
    localparam signed [DATA_WIDTH-1:0] MPP_THRESHOLD = 32'h0000_0CCC; // ~0.05 in Q16.16
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            v_ref <= 0;
            pv_i_old <= 0;
            pv_v_old <= 0;
            delta_i <= 0;
            delta_v <= 0;
            conductance <= 0;
            inc_conductance <= 0;
            neg_conductance <= 0;
            first_sample <= 1;
            calc_step <= 0;
            valid <= 0;
            at_mpp <= 0;
            math_start <= 0;
            math_op_a <= 0;
            math_op_b <= 0;
            math_operation <= 0;
        end else begin
            case (state)
                IDLE: begin
                    valid <= 0;
                    at_mpp <= 0;
                    if (enable) begin
                        if (first_sample) begin
                            // First sample: just store values and initialize v_ref
                            pv_i_old <= pv_current;
                            pv_v_old <= pv_voltage;
                            v_ref <= pv_voltage;
                            first_sample <= 0;
                            valid <= 1;
                            state <= WAIT;
                        end else begin
                            state <= CALC;
                            calc_step <= 0;
                        end
                    end
                end
                
                CALC: begin
                    case (calc_step)
                        // Step 0: Calculate delta_i = pv_current - pv_i_old
                        4'd0: begin
                            math_op_a <= pv_current;
                            math_op_b <= pv_i_old;
                            math_operation <= 3'd1; // SUB
                            math_start <= 1;
                            calc_step <= calc_step + 1;
                        end
                        4'd1: begin
                            math_start <= 0;
                            if (math_done) begin
                                delta_i <= math_result;
                                calc_step <= calc_step + 1;
                            end
                        end
                        
                        // Step 2: Calculate delta_v = pv_voltage - pv_v_old
                        4'd2: begin
                            math_op_a <= pv_voltage;
                            math_op_b <= pv_v_old;
                            math_operation <= 3'd1; // SUB
                            math_start <= 1;
                            calc_step <= calc_step + 1;
                        end
                        4'd3: begin
                            math_start <= 0;
                            if (math_done) begin
                                delta_v <= math_result;
                                calc_step <= calc_step + 1;
                            end
                        end
                        
                        // Step 4: Calculate conductance = pv_current / pv_voltage
                        4'd4: begin
                            if (pv_voltage != 0) begin
                                math_op_a <= pv_current;
                                math_op_b <= pv_voltage;
                                math_operation <= 3'd3; // DIV
                                math_start <= 1;
                                calc_step <= calc_step + 1;
                            end else begin
                                conductance <= 0;
                                calc_step <= 4'd6; // Skip to next calculation
                            end
                        end
                        4'd5: begin
                            math_start <= 0;
                            if (math_done) begin
                                conductance <= math_result;
                                calc_step <= calc_step + 1;
                            end
                        end
                        
                        // Step 6: Calculate neg_conductance = -conductance
                        4'd6: begin
                            neg_conductance <= -conductance;
                            calc_step <= calc_step + 1;
                        end
                        
                        // Step 7: Calculate inc_conductance = delta_i / delta_v
                        4'd7: begin
                            if (delta_v != 0) begin
                                math_op_a <= delta_i;
                                math_op_b <= delta_v;
                                math_operation <= 3'd3; // DIV
                                math_start <= 1;
                                calc_step <= calc_step + 1;
                            end else begin
                                // delta_v = 0: Use delta_i sign for direction
                                if (delta_i > 0)
                                    inc_conductance <= 32'h7FFF_FFFF; // Large positive
                                else if (delta_i < 0)
                                    inc_conductance <= 32'h8000_0000; // Large negative
                                else
                                    inc_conductance <= 0;
                                calc_step <= 4'd9; // Skip to comparison
                            end
                        end
                        4'd8: begin
                            math_start <= 0;
                            if (math_done) begin
                                inc_conductance <= math_result;
                                calc_step <= calc_step + 1;
                            end
                        end
                        
                        // Step 9: Compare inc_conductance with neg_conductance
                        4'd9: begin
                            math_op_a <= inc_conductance;
                            math_op_b <= neg_conductance;
                            math_operation <= 3'd4; // COMPARE
                            math_start <= 1;
                            calc_step <= calc_step + 1;
                        end
                        4'd10: begin
                            math_start <= 0;
                            if (math_done) begin
                                calc_step <= calc_step + 1;
                                state <= UPDATE;
                            end
                        end
                        
                        default: begin
                            state <= UPDATE;
                        end
                    endcase
                end
                
                UPDATE: begin
                    // Check if at MPP (inc_conductance ≈ neg_conductance)
                    math_op_a <= inc_conductance;
                    math_op_b <= neg_conductance;
                    math_operation <= 3'd1; // SUB to get difference
                    
                    // Determine voltage adjustment based on comparison
                    if (math_compare == 2'b01) begin
                        // inc_conductance > neg_conductance: Left of MPP, increase voltage
                        if (v_ref + step_size <= v_max)
                            v_ref <= v_ref + step_size;
                        else
                            v_ref <= v_max;
                        at_mpp <= 0;
                    end else if (math_compare == 2'b10) begin
                        // inc_conductance < neg_conductance: Right of MPP, decrease voltage
                        if (v_ref - step_size >= v_min)
                            v_ref <= v_ref - step_size;
                        else
                            v_ref <= v_min;
                        at_mpp <= 0;
                    end else begin
                        // At or very close to MPP
                        at_mpp <= 1;
                    end
                    
                    // Update old values for next iteration
                    pv_i_old <= pv_current;
                    pv_v_old <= pv_voltage;
                    
                    valid <= 1;
                    state <= WAIT;
                end
                
                WAIT: begin
                    valid <= 0;
                    if (!enable) begin
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule