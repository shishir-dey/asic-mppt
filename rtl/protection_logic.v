`timescale 1ns / 1ps

//==============================================================================
// Protection Logic Module
//==============================================================================
// This module implements comprehensive protection features for the MPPT system:
//   - Overvoltage protection (battery and solar)
//   - Undervoltage protection (battery)
//   - Overcurrent protection (battery and solar)
//   - Overtemperature protection
//   - Backflow protection
//   - Thermal management (fan control)
//
// Protection actions:
//   - Shutdown: Complete system shutdown
//   - Fan drive: Activate cooling fan
//   - Backflow protection: Prevent reverse current flow
//==============================================================================

module protection_logic #(
    parameter DATA_WIDTH = 32  // Q16.16 fixed-point format
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    
    // Measured values (Q16.16 format)
    input wire signed [DATA_WIDTH-1:0] battery_voltage,
    input wire signed [DATA_WIDTH-1:0] battery_current,
    input wire signed [DATA_WIDTH-1:0] solar_voltage,
    input wire signed [DATA_WIDTH-1:0] solar_current,
    input wire signed [DATA_WIDTH-1:0] temperature_1,
    input wire signed [DATA_WIDTH-1:0] temperature_2,
    
    // Protection thresholds (Q16.16 format)
    input wire signed [DATA_WIDTH-1:0] batt_v_max,
    input wire signed [DATA_WIDTH-1:0] batt_v_min,
    input wire signed [DATA_WIDTH-1:0] batt_i_max,
    input wire signed [DATA_WIDTH-1:0] solar_v_max,
    input wire signed [DATA_WIDTH-1:0] solar_i_max,
    input wire signed [DATA_WIDTH-1:0] temp_max,
    input wire signed [DATA_WIDTH-1:0] temp_fan_on,
    input wire signed [DATA_WIDTH-1:0] temp_fan_off,
    
    // Protection outputs
    output reg shutdown,
    output reg fan_drive,
    output reg backflow_protection,
    
    // Status flags
    output reg overvoltage_fault,
    output reg undervoltage_fault,
    output reg overcurrent_fault,
    output reg overtemperature_fault,
    output reg backflow_fault
);

    // Default protection thresholds (Q16.16 format)
    // Battery: 48V nominal, 60V max, 40V min
    localparam signed [DATA_WIDTH-1:0] DEFAULT_BATT_V_MAX = 32'h003C_0000; // 60V
    localparam signed [DATA_WIDTH-1:0] DEFAULT_BATT_V_MIN = 32'h0028_0000; // 40V
    localparam signed [DATA_WIDTH-1:0] DEFAULT_BATT_I_MAX = 32'h0014_0000; // 20A
    localparam signed [DATA_WIDTH-1:0] DEFAULT_SOLAR_V_MAX = 32'h003C_0000; // 60V
    localparam signed [DATA_WIDTH-1:0] DEFAULT_SOLAR_I_MAX = 32'h0014_0000; // 20A
    localparam signed [DATA_WIDTH-1:0] DEFAULT_TEMP_MAX = 32'h0050_0000; // 80°C
    localparam signed [DATA_WIDTH-1:0] DEFAULT_TEMP_FAN_ON = 32'h002D_0000; // 45°C
    localparam signed [DATA_WIDTH-1:0] DEFAULT_TEMP_FAN_OFF = 32'h0028_0000; // 40°C
    
    // Internal threshold registers (use defaults if not configured)
    reg signed [DATA_WIDTH-1:0] batt_v_max_int;
    reg signed [DATA_WIDTH-1:0] batt_v_min_int;
    reg signed [DATA_WIDTH-1:0] batt_i_max_int;
    reg signed [DATA_WIDTH-1:0] solar_v_max_int;
    reg signed [DATA_WIDTH-1:0] solar_i_max_int;
    reg signed [DATA_WIDTH-1:0] temp_max_int;
    reg signed [DATA_WIDTH-1:0] temp_fan_on_int;
    reg signed [DATA_WIDTH-1:0] temp_fan_off_int;
    
    // Fault detection counters (debouncing)
    reg [7:0] overvoltage_counter;
    reg [7:0] undervoltage_counter;
    reg [7:0] overcurrent_counter;
    reg [7:0] overtemperature_counter;
    reg [7:0] backflow_counter;
    
    localparam FAULT_THRESHOLD = 8'd10; // 10 consecutive samples to trigger fault
    localparam CLEAR_THRESHOLD = 8'd50; // 50 consecutive good samples to clear fault
    
    reg [7:0] clear_counter;
    reg fault_active;
    
    // Temperature for fan control (use maximum of both sensors)
    reg signed [DATA_WIDTH-1:0] max_temperature;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize thresholds
            batt_v_max_int <= DEFAULT_BATT_V_MAX;
            batt_v_min_int <= DEFAULT_BATT_V_MIN;
            batt_i_max_int <= DEFAULT_BATT_I_MAX;
            solar_v_max_int <= DEFAULT_SOLAR_V_MAX;
            solar_i_max_int <= DEFAULT_SOLAR_I_MAX;
            temp_max_int <= DEFAULT_TEMP_MAX;
            temp_fan_on_int <= DEFAULT_TEMP_FAN_ON;
            temp_fan_off_int <= DEFAULT_TEMP_FAN_OFF;
            
            // Initialize outputs
            shutdown <= 0;
            fan_drive <= 0;
            backflow_protection <= 0;
            
            // Initialize fault flags
            overvoltage_fault <= 0;
            undervoltage_fault <= 0;
            overcurrent_fault <= 0;
            overtemperature_fault <= 0;
            backflow_fault <= 0;
            
            // Initialize counters
            overvoltage_counter <= 0;
            undervoltage_counter <= 0;
            overcurrent_counter <= 0;
            overtemperature_counter <= 0;
            backflow_counter <= 0;
            clear_counter <= 0;
            fault_active <= 0;
            
            max_temperature <= 0;
        end else begin
            // Update thresholds if provided (non-zero values)
            if (batt_v_max != 0) batt_v_max_int <= batt_v_max;
            if (batt_v_min != 0) batt_v_min_int <= batt_v_min;
            if (batt_i_max != 0) batt_i_max_int <= batt_i_max;
            if (solar_v_max != 0) solar_v_max_int <= solar_v_max;
            if (solar_i_max != 0) solar_i_max_int <= solar_i_max;
            if (temp_max != 0) temp_max_int <= temp_max;
            if (temp_fan_on != 0) temp_fan_on_int <= temp_fan_on;
            if (temp_fan_off != 0) temp_fan_off_int <= temp_fan_off;
            
            if (enable) begin
                // Calculate maximum temperature
                max_temperature <= (temperature_1 > temperature_2) ? temperature_1 : temperature_2;
                
                // ============================================================
                // Overvoltage Detection
                // ============================================================
                if (battery_voltage > batt_v_max_int || solar_voltage > solar_v_max_int) begin
                    if (overvoltage_counter < FAULT_THRESHOLD)
                        overvoltage_counter <= overvoltage_counter + 1;
                    else
                        overvoltage_fault <= 1;
                end else begin
                    if (overvoltage_counter > 0)
                        overvoltage_counter <= overvoltage_counter - 1;
                end
                
                // ============================================================
                // Undervoltage Detection
                // ============================================================
                if (battery_voltage < batt_v_min_int && battery_voltage > 0) begin
                    if (undervoltage_counter < FAULT_THRESHOLD)
                        undervoltage_counter <= undervoltage_counter + 1;
                    else
                        undervoltage_fault <= 1;
                end else begin
                    if (undervoltage_counter > 0)
                        undervoltage_counter <= undervoltage_counter - 1;
                end
                
                // ============================================================
                // Overcurrent Detection
                // ============================================================
                if (battery_current > batt_i_max_int || solar_current > solar_i_max_int) begin
                    if (overcurrent_counter < FAULT_THRESHOLD)
                        overcurrent_counter <= overcurrent_counter + 1;
                    else
                        overcurrent_fault <= 1;
                end else begin
                    if (overcurrent_counter > 0)
                        overcurrent_counter <= overcurrent_counter - 1;
                end
                
                // ============================================================
                // Overtemperature Detection
                // ============================================================
                if (max_temperature > temp_max_int) begin
                    if (overtemperature_counter < FAULT_THRESHOLD)
                        overtemperature_counter <= overtemperature_counter + 1;
                    else
                        overtemperature_fault <= 1;
                end else begin
                    if (overtemperature_counter > 0)
                        overtemperature_counter <= overtemperature_counter - 1;
                end
                
                // ============================================================
                // Backflow Detection (negative battery current)
                // ============================================================
                if (battery_current < 0) begin
                    if (backflow_counter < FAULT_THRESHOLD)
                        backflow_counter <= backflow_counter + 1;
                    else begin
                        backflow_fault <= 1;
                        backflow_protection <= 1;
                    end
                end else begin
                    if (backflow_counter > 0)
                        backflow_counter <= backflow_counter - 1;
                    else
                        backflow_protection <= 0;
                end
                
                // ============================================================
                // Shutdown Logic
                // ============================================================
                fault_active <= overvoltage_fault | undervoltage_fault | 
                               overcurrent_fault | overtemperature_fault;
                
                if (fault_active) begin
                    shutdown <= 1;
                    clear_counter <= 0;
                end else begin
                    // Clear shutdown after fault conditions are resolved
                    if (clear_counter < CLEAR_THRESHOLD)
                        clear_counter <= clear_counter + 1;
                    else begin
                        shutdown <= 0;
                        // Clear fault flags
                        overvoltage_fault <= 0;
                        undervoltage_fault <= 0;
                        overcurrent_fault <= 0;
                        overtemperature_fault <= 0;
                    end
                end
                
                // ============================================================
                // Fan Control (Hysteresis)
                // ============================================================
                if (max_temperature > temp_fan_on_int) begin
                    fan_drive <= 1;
                end else if (max_temperature < temp_fan_off_int) begin
                    fan_drive <= 0;
                end
                // Maintain current state if between thresholds (hysteresis)
                
            end else begin
                // System disabled - safe state
                shutdown <= 1;
                fan_drive <= 0;
                backflow_protection <= 1;
            end
        end
    end

endmodule