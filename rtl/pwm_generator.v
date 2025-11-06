`timescale 1ns / 1ps

//==============================================================================
// PWM Generator for DC-DC Converter Control
//==============================================================================
// This module generates a PWM signal to control the DC-DC converter based on
// the voltage reference from the MPPT algorithm. The duty cycle is calculated
// to achieve the desired output voltage.
//
// Features:
//   - Configurable PWM frequency (default 100 kHz)
//   - 10-bit duty cycle resolution (0-1023)
//   - Dead-time insertion for complementary outputs
//   - Soft-start capability
//   - Emergency shutdown input
//==============================================================================

module pwm_generator #(
    parameter CLK_FREQ = 50_000_000,    // 50 MHz system clock
    parameter PWM_FREQ = 100_000,       // 100 kHz PWM frequency
    parameter DATA_WIDTH = 32,          // Q16.16 fixed-point format
    parameter DUTY_BITS = 10            // 10-bit duty cycle resolution
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire shutdown,
    
    // Voltage reference from MPPT (Q16.16 format)
    input wire signed [DATA_WIDTH-1:0] v_ref,
    
    // Measured voltages for feedback (Q16.16 format)
    input wire signed [DATA_WIDTH-1:0] v_in,
    input wire signed [DATA_WIDTH-1:0] v_out,
    
    // Configuration
    input wire [DUTY_BITS-1:0] duty_max,
    input wire [DUTY_BITS-1:0] duty_min,
    input wire [7:0] dead_time,
    input wire soft_start_enable,
    
    // PWM outputs
    output reg pwm_high,
    output reg pwm_low,
    output reg [DUTY_BITS-1:0] duty_cycle
);

    // PWM period calculation
    localparam PWM_PERIOD = CLK_FREQ / PWM_FREQ;
    localparam COUNTER_BITS = $clog2(PWM_PERIOD);
    
    // Default limits
    localparam [DUTY_BITS-1:0] DEFAULT_DUTY_MAX = 10'd900; // 90% max
    localparam [DUTY_BITS-1:0] DEFAULT_DUTY_MIN = 10'd50;  // 5% min
    localparam [7:0] DEFAULT_DEAD_TIME = 8'd10;            // 10 clock cycles
    
    // Internal registers
    reg [COUNTER_BITS-1:0] pwm_counter;
    reg [DUTY_BITS-1:0] duty_compare;
    reg [DUTY_BITS-1:0] duty_max_int;
    reg [DUTY_BITS-1:0] duty_min_int;
    reg [7:0] dead_time_int;
    reg [7:0] dead_time_counter;
    
    // Soft-start
    reg [DUTY_BITS-1:0] soft_start_duty;
    reg [15:0] soft_start_counter;
    reg soft_start_active;
    localparam SOFT_START_STEPS = 16'd1000; // Ramp up over 1000 PWM cycles
    
    // Duty cycle calculation
    reg signed [DATA_WIDTH-1:0] v_error;
    reg signed [DATA_WIDTH-1:0] duty_calc;
    
    // Simple proportional controller for duty cycle
    // duty = (v_ref / v_in) * 1024 (for buck converter)
    wire signed [2*DATA_WIDTH-1:0] duty_mult;
    assign duty_mult = v_ref * 32'h0000_0400; // Multiply by 1024
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 0;
            duty_cycle <= 0;
            duty_compare <= 0;
            pwm_high <= 0;
            pwm_low <= 0;
            dead_time_counter <= 0;
            
            duty_max_int <= DEFAULT_DUTY_MAX;
            duty_min_int <= DEFAULT_DUTY_MIN;
            dead_time_int <= DEFAULT_DEAD_TIME;
            
            soft_start_duty <= 0;
            soft_start_counter <= 0;
            soft_start_active <= 0;
            
            v_error <= 0;
            duty_calc <= 0;
        end else begin
            // Update configuration
            if (duty_max != 0) duty_max_int <= duty_max;
            if (duty_min != 0) duty_min_int <= duty_min;
            if (dead_time != 0) dead_time_int <= dead_time;
            
            if (enable && !shutdown) begin
                // ============================================================
                // Duty Cycle Calculation
                // ============================================================
                if (v_in > 0) begin
                    // Calculate duty cycle: duty = (v_ref / v_in) * 1024
                    duty_calc <= duty_mult[DATA_WIDTH+9:10];
                    
                    // Limit duty cycle to configured range
                    if (duty_calc[DUTY_BITS-1:0] > duty_max_int)
                        duty_compare <= duty_max_int;
                    else if (duty_calc[DUTY_BITS-1:0] < duty_min_int)
                        duty_compare <= duty_min_int;
                    else
                        duty_compare <= duty_calc[DUTY_BITS-1:0];
                end else begin
                    duty_compare <= duty_min_int;
                end
                
                // ============================================================
                // Soft-Start Logic
                // ============================================================
                if (soft_start_enable && soft_start_active) begin
                    if (soft_start_counter < SOFT_START_STEPS) begin
                        // Gradually increase duty cycle
                        soft_start_duty <= (duty_compare * soft_start_counter) / SOFT_START_STEPS;
                        duty_cycle <= soft_start_duty;
                        
                        if (pwm_counter == 0)
                            soft_start_counter <= soft_start_counter + 1;
                    end else begin
                        // Soft-start complete
                        soft_start_active <= 0;
                        duty_cycle <= duty_compare;
                    end
                end else begin
                    duty_cycle <= duty_compare;
                    if (soft_start_enable && pwm_counter == 0)
                        soft_start_active <= 1;
                end
                
                // ============================================================
                // PWM Counter and Output Generation
                // ============================================================
                if (pwm_counter < PWM_PERIOD - 1) begin
                    pwm_counter <= pwm_counter + 1;
                end else begin
                    pwm_counter <= 0;
                end
                
                // Scale duty cycle to PWM period
                // duty_scaled = (duty_cycle * PWM_PERIOD) / 1024
                if (pwm_counter < ((duty_cycle * PWM_PERIOD) >> DUTY_BITS)) begin
                    pwm_high <= 1;
                end else begin
                    pwm_high <= 0;
                end
                
                // ============================================================
                // Dead-Time Insertion for Complementary Output
                // ============================================================
                if (pwm_high) begin
                    pwm_low <= 0;
                    dead_time_counter <= 0;
                end else begin
                    if (dead_time_counter < dead_time_int) begin
                        pwm_low <= 0;
                        dead_time_counter <= dead_time_counter + 1;
                    end else begin
                        pwm_low <= 1;
                    end
                end
                
            end else begin
                // Disabled or shutdown - safe state
                pwm_high <= 0;
                pwm_low <= 0;
                pwm_counter <= 0;
                duty_cycle <= 0;
                soft_start_counter <= 0;
                soft_start_active <= 0;
            end
        end
    end

endmodule