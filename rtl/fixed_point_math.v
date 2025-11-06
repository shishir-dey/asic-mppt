`timescale 1ns / 1ps

//==============================================================================
// Fixed-Point Arithmetic Module for MPPT Calculations
//==============================================================================
// This module provides fixed-point arithmetic operations needed for the
// Incremental Conductance MPPT algorithm. Uses Q16.16 format (16 integer bits,
// 16 fractional bits) for high precision calculations.
//==============================================================================

module fixed_point_math #(
    parameter INT_BITS = 16,
    parameter FRAC_BITS = 16,
    parameter TOTAL_BITS = INT_BITS + FRAC_BITS
)(
    input wire clk,
    input wire rst_n,
    
    // Operands
    input wire signed [TOTAL_BITS-1:0] operand_a,
    input wire signed [TOTAL_BITS-1:0] operand_b,
    
    // Operation select
    input wire [2:0] operation,  // 0:add, 1:sub, 2:mul, 3:div, 4:compare
    input wire start,
    
    // Results
    output reg signed [TOTAL_BITS-1:0] result,
    output reg done,
    output reg overflow,
    output reg [1:0] compare_result  // 00:equal, 01:a>b, 10:a<b
);

    // Operation codes
    localparam OP_ADD = 3'd0;
    localparam OP_SUB = 3'd1;
    localparam OP_MUL = 3'd2;
    localparam OP_DIV = 3'd3;
    localparam OP_CMP = 3'd4;
    
    // Internal signals for division
    reg [5:0] div_counter;
    reg signed [2*TOTAL_BITS-1:0] dividend;
    reg signed [TOTAL_BITS-1:0] divisor;
    reg div_active;
    
    // Multiplication intermediate
    wire signed [2*TOTAL_BITS-1:0] mul_result;
    assign mul_result = operand_a * operand_b;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result <= 0;
            done <= 0;
            overflow <= 0;
            compare_result <= 2'b00;
            div_counter <= 0;
            div_active <= 0;
            dividend <= 0;
            divisor <= 0;
        end else begin
            if (start && !div_active) begin
                done <= 0;
                overflow <= 0;
                
                case (operation)
                    OP_ADD: begin
                        result <= operand_a + operand_b;
                        done <= 1;
                        // Check for overflow
                        if ((operand_a[TOTAL_BITS-1] == operand_b[TOTAL_BITS-1]) && 
                            (result[TOTAL_BITS-1] != operand_a[TOTAL_BITS-1]))
                            overflow <= 1;
                    end
                    
                    OP_SUB: begin
                        result <= operand_a - operand_b;
                        done <= 1;
                        // Check for overflow
                        if ((operand_a[TOTAL_BITS-1] != operand_b[TOTAL_BITS-1]) && 
                            (result[TOTAL_BITS-1] != operand_a[TOTAL_BITS-1]))
                            overflow <= 1;
                    end
                    
                    OP_MUL: begin
                        // Shift right by FRAC_BITS to maintain fixed-point format
                        result <= mul_result[TOTAL_BITS+FRAC_BITS-1:FRAC_BITS];
                        done <= 1;
                        // Check for overflow
                        if (mul_result[2*TOTAL_BITS-1:TOTAL_BITS+FRAC_BITS] != 
                            {(TOTAL_BITS-FRAC_BITS){mul_result[2*TOTAL_BITS-1]}})
                            overflow <= 1;
                    end
                    
                    OP_DIV: begin
                        if (operand_b == 0) begin
                            result <= {TOTAL_BITS{1'b0}};
                            overflow <= 1;
                            done <= 1;
                        end else begin
                            // Initialize division
                            dividend <= {{TOTAL_BITS{operand_a[TOTAL_BITS-1]}}, operand_a} << FRAC_BITS;
                            divisor <= operand_b;
                            div_counter <= TOTAL_BITS;
                            div_active <= 1;
                        end
                    end
                    
                    OP_CMP: begin
                        if (operand_a == operand_b)
                            compare_result <= 2'b00;
                        else if (operand_a > operand_b)
                            compare_result <= 2'b01;
                        else
                            compare_result <= 2'b10;
                        done <= 1;
                    end
                    
                    default: begin
                        result <= 0;
                        done <= 1;
                    end
                endcase
            end else if (div_active) begin
                // Non-restoring division algorithm
                if (div_counter > 0) begin
                    if (dividend[2*TOTAL_BITS-1] == divisor[TOTAL_BITS-1]) begin
                        dividend <= (dividend << 1) - {divisor, {TOTAL_BITS{1'b0}}};
                    end else begin
                        dividend <= (dividend << 1) + {divisor, {TOTAL_BITS{1'b0}}};
                    end
                    div_counter <= div_counter - 1;
                end else begin
                    result <= dividend[TOTAL_BITS-1:0];
                    div_active <= 0;
                    done <= 1;
                end
            end else begin
                done <= 0;
            end
        end
    end

endmodule