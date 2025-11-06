`timescale 1ns / 1ps

//==============================================================================
// MPPT Controller Top-Level Module
//==============================================================================
// Complete ASIC implementation of Maximum Power Point Tracking controller
// using the Incremental Conductance algorithm.
//
// Features:
//   - Incremental Conductance MPPT algorithm
//   - I2C slave interface for configuration
//   - SPI slave interface for fast data readout
//   - UART/Modbus RTU interface for remote monitoring
//   - Comprehensive protection logic
//   - PWM generation for DC-DC converter control
//   - Multi-channel ADC interface with filtering
//==============================================================================

module mppt_controller #(
    parameter CLK_FREQ = 50_000_000,
    parameter DATA_WIDTH = 32,
    parameter I2C_ADDR = 7'h50,
    parameter MODBUS_ADDR = 8'h01
)(
    // Clock and reset
    input wire clk,
    input wire rst_n,
    
    // Analog inputs (12-bit ADC)
    input wire [11:0] battery_voltage_sense,
    input wire [11:0] battery_current_sense,
    input wire [11:0] solar_voltage_sense,
    input wire [11:0] solar_current_sense,
    input wire [11:0] temperature_sense_1,
    input wire [11:0] temperature_sense_2,
    
    // Digital outputs
    output wire shutdown,
    output wire fan_drive,
    output wire backflow_protection,
    output wire pwm_high,
    output wire pwm_low,
    
    // UART interface for Modbus
    input wire uart_rx,
    output wire uart_tx,
    
    // I2C interface
    inout wire i2c_sda,
    input wire i2c_scl,
    
    // SPI interface
    input wire spi_sck,
    input wire spi_mosi,
    output wire spi_miso,
    input wire spi_ss
);

    //==========================================================================
    // Internal Signals
    //==========================================================================
    
    // ADC interface outputs
    wire signed [DATA_WIDTH-1:0] battery_voltage;
    wire signed [DATA_WIDTH-1:0] battery_current;
    wire signed [DATA_WIDTH-1:0] solar_voltage;
    wire signed [DATA_WIDTH-1:0] solar_current;
    wire signed [DATA_WIDTH-1:0] temperature_1;
    wire signed [DATA_WIDTH-1:0] temperature_2;
    wire adc_data_valid;
    
    // MPPT core outputs
    wire signed [DATA_WIDTH-1:0] mppt_v_ref;
    wire mppt_valid;
    wire mppt_at_mpp;
    wire [1:0] mppt_state;
    
    // Protection logic outputs
    wire protection_shutdown;
    wire protection_fan;
    wire protection_backflow;
    wire overvoltage_fault;
    wire undervoltage_fault;
    wire overcurrent_fault;
    wire overtemperature_fault;
    wire backflow_fault;
    
    // PWM generator outputs
    wire [9:0] pwm_duty_cycle;
    
    // Register bank
    reg [7:0] registers [0:255];
    
    // Control registers
    reg mppt_enable;
    reg soft_start_enable;
    reg [7:0] control_reg;
    reg [7:0] status_reg;
    
    // MPPT configuration (Q16.16 format)
    reg signed [DATA_WIDTH-1:0] mppt_step_size;
    reg signed [DATA_WIDTH-1:0] mppt_v_max;
    reg signed [DATA_WIDTH-1:0] mppt_v_min;
    
    // Protection thresholds (Q16.16 format)
    reg signed [DATA_WIDTH-1:0] batt_v_max_thresh;
    reg signed [DATA_WIDTH-1:0] batt_v_min_thresh;
    reg signed [DATA_WIDTH-1:0] batt_i_max_thresh;
    reg signed [DATA_WIDTH-1:0] solar_v_max_thresh;
    reg signed [DATA_WIDTH-1:0] solar_i_max_thresh;
    reg signed [DATA_WIDTH-1:0] temp_max_thresh;
    reg signed [DATA_WIDTH-1:0] temp_fan_on_thresh;
    reg signed [DATA_WIDTH-1:0] temp_fan_off_thresh;
    
    // PWM configuration
    reg [9:0] pwm_duty_max;
    reg [9:0] pwm_duty_min;
    reg [7:0] pwm_dead_time;
    
    // ADC calibration
    reg signed [DATA_WIDTH-1:0] batt_v_scale;
    reg signed [DATA_WIDTH-1:0] batt_v_offset;
    reg signed [DATA_WIDTH-1:0] batt_i_scale;
    reg signed [DATA_WIDTH-1:0] batt_i_offset;
    reg signed [DATA_WIDTH-1:0] solar_v_scale;
    reg signed [DATA_WIDTH-1:0] solar_v_offset;
    reg signed [DATA_WIDTH-1:0] solar_i_scale;
    reg signed [DATA_WIDTH-1:0] solar_i_offset;
    
    // Communication interface signals
    wire [7:0] i2c_reg_addr;
    wire [7:0] i2c_reg_wdata;
    wire i2c_reg_write;
    wire [7:0] i2c_reg_rdata;
    wire i2c_reg_read;
    
    wire [7:0] spi_reg_addr;
    wire [7:0] spi_reg_rdata;
    wire spi_reg_read;
    
    wire [7:0] uart_reg_addr;
    wire [7:0] uart_reg_wdata;
    wire uart_reg_write;
    wire [7:0] uart_reg_rdata;
    wire uart_reg_read;
    
    // Combined register interface
    wire [7:0] reg_addr;
    wire [7:0] reg_wdata;
    wire reg_write;
    wire [7:0] reg_rdata;
    wire reg_read;
    
    // Arbitrate between communication interfaces (priority: I2C > SPI > UART)
    assign reg_addr = i2c_reg_write ? i2c_reg_addr :
                      i2c_reg_read ? i2c_reg_addr :
                      spi_reg_read ? spi_reg_addr :
                      uart_reg_write ? uart_reg_addr :
                      uart_reg_read ? uart_reg_addr : 8'h00;
    
    assign reg_wdata = i2c_reg_write ? i2c_reg_wdata :
                       uart_reg_write ? uart_reg_wdata : 8'h00;
    
    assign reg_write = i2c_reg_write | uart_reg_write;
    assign reg_read = i2c_reg_read | spi_reg_read | uart_reg_read;
    
    assign i2c_reg_rdata = reg_rdata;
    assign spi_reg_rdata = reg_rdata;
    assign uart_reg_rdata = reg_rdata;
    
    //==========================================================================
    // Module Instantiations
    //==========================================================================
    
    // ADC Interface
    adc_interface #(
        .ADC_BITS(12),
        .DATA_WIDTH(DATA_WIDTH),
        .FILTER_DEPTH(8)
    ) adc_inst (
        .clk(clk),
        .rst_n(rst_n),
        .battery_voltage_adc(battery_voltage_sense),
        .battery_current_adc(battery_current_sense),
        .solar_voltage_adc(solar_voltage_sense),
        .solar_current_adc(solar_current_sense),
        .temperature_1_adc(temperature_sense_1),
        .temperature_2_adc(temperature_sense_2),
        .batt_v_scale(batt_v_scale),
        .batt_v_offset(batt_v_offset),
        .batt_i_scale(batt_i_scale),
        .batt_i_offset(batt_i_offset),
        .solar_v_scale(solar_v_scale),
        .solar_v_offset(solar_v_offset),
        .solar_i_scale(solar_i_scale),
        .solar_i_offset(solar_i_offset),
        .battery_voltage(battery_voltage),
        .battery_current(battery_current),
        .solar_voltage(solar_voltage),
        .solar_current(solar_current),
        .temperature_1(temperature_1),
        .temperature_2(temperature_2),
        .data_valid(adc_data_valid)
    );
    
    // MPPT IC Core Algorithm
    mppt_ic_core #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_BITS(16)
    ) mppt_core (
        .clk(clk),
        .rst_n(rst_n),
        .enable(mppt_enable && !protection_shutdown),
        .pv_current(solar_current),
        .pv_voltage(solar_voltage),
        .step_size(mppt_step_size),
        .v_max(mppt_v_max),
        .v_min(mppt_v_min),
        .v_ref(mppt_v_ref),
        .valid(mppt_valid),
        .at_mpp(mppt_at_mpp),
        .state(mppt_state)
    );
    
    // Protection Logic
    protection_logic #(
        .DATA_WIDTH(DATA_WIDTH)
    ) protection (
        .clk(clk),
        .rst_n(rst_n),
        .enable(mppt_enable),
        .battery_voltage(battery_voltage),
        .battery_current(battery_current),
        .solar_voltage(solar_voltage),
        .solar_current(solar_current),
        .temperature_1(temperature_1),
        .temperature_2(temperature_2),
        .batt_v_max(batt_v_max_thresh),
        .batt_v_min(batt_v_min_thresh),
        .batt_i_max(batt_i_max_thresh),
        .solar_v_max(solar_v_max_thresh),
        .solar_i_max(solar_i_max_thresh),
        .temp_max(temp_max_thresh),
        .temp_fan_on(temp_fan_on_thresh),
        .temp_fan_off(temp_fan_off_thresh),
        .shutdown(protection_shutdown),
        .fan_drive(protection_fan),
        .backflow_protection(protection_backflow),
        .overvoltage_fault(overvoltage_fault),
        .undervoltage_fault(undervoltage_fault),
        .overcurrent_fault(overcurrent_fault),
        .overtemperature_fault(overtemperature_fault),
        .backflow_fault(backflow_fault)
    );
    
    // PWM Generator
    pwm_generator #(
        .CLK_FREQ(CLK_FREQ),
        .PWM_FREQ(100_000),
        .DATA_WIDTH(DATA_WIDTH),
        .DUTY_BITS(10)
    ) pwm_gen (
        .clk(clk),
        .rst_n(rst_n),
        .enable(mppt_enable && !protection_shutdown),
        .shutdown(protection_shutdown),
        .v_ref(mppt_v_ref),
        .v_in(solar_voltage),
        .v_out(battery_voltage),
        .duty_max(pwm_duty_max),
        .duty_min(pwm_duty_min),
        .dead_time(pwm_dead_time),
        .soft_start_enable(soft_start_enable),
        .pwm_high(pwm_high),
        .pwm_low(pwm_low),
        .duty_cycle(pwm_duty_cycle)
    );
    
    // I2C Slave Interface
    i2c_slave #(
        .SLAVE_ADDR(I2C_ADDR)
    ) i2c (
        .clk(clk),
        .rst_n(rst_n),
        .sda(i2c_sda),
        .scl(i2c_scl),
        .reg_addr(i2c_reg_addr),
        .reg_wdata(i2c_reg_wdata),
        .reg_write(i2c_reg_write),
        .reg_rdata(i2c_reg_rdata),
        .reg_read(i2c_reg_read)
    );
    
    // SPI Slave Interface
    spi_slave spi (
        .clk(clk),
        .rst_n(rst_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_ss(spi_ss),
        .reg_addr(spi_reg_addr),
        .reg_rdata(spi_reg_rdata),
        .reg_read(spi_reg_read)
    );
    
    // UART/Modbus Interface
    uart_modbus #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(9600),
        .MODBUS_ADDR(MODBUS_ADDR)
    ) uart (
        .clk(clk),
        .rst_n(rst_n),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .reg_addr(uart_reg_addr),
        .reg_wdata(uart_reg_wdata),
        .reg_write(uart_reg_write),
        .reg_rdata(uart_reg_rdata),
        .reg_read(uart_reg_read),
        .frame_error(),
        .crc_error()
    );
    
    //==========================================================================
    // Register Bank Management
    //==========================================================================
    
    integer i;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize all registers
            for (i = 0; i < 256; i = i + 1) begin
                registers[i] <= 8'h00;
            end
            
            // Default configuration
            mppt_enable <= 0;
            soft_start_enable <= 1;
            control_reg <= 8'h00;
            
            // Default MPPT parameters (Q16.16)
            mppt_step_size <= 32'h0000_0CCC;  // 0.05V
            mppt_v_max <= 32'h001E_0000;      // 30V
            mppt_v_min <= 32'h000C_0000;      // 12V
            
            // Default protection thresholds
            batt_v_max_thresh <= 32'h003C_0000;  // 60V
            batt_v_min_thresh <= 32'h0028_0000;  // 40V
            batt_i_max_thresh <= 32'h0014_0000;  // 20A
            solar_v_max_thresh <= 32'h003C_0000; // 60V
            solar_i_max_thresh <= 32'h0014_0000; // 20A
            temp_max_thresh <= 32'h0050_0000;    // 80°C
            temp_fan_on_thresh <= 32'h002D_0000; // 45°C
            temp_fan_off_thresh <= 32'h0028_0000;// 40°C
            
            // Default PWM configuration
            pwm_duty_max <= 10'd900;  // 90%
            pwm_duty_min <= 10'd50;   // 5%
            pwm_dead_time <= 8'd10;
            
            // Default calibration (zero offset, unity scale)
            batt_v_scale <= 0;
            batt_v_offset <= 0;
            batt_i_scale <= 0;
            batt_i_offset <= 0;
            solar_v_scale <= 0;
            solar_v_offset <= 0;
            solar_i_scale <= 0;
            solar_i_offset <= 0;
        end else begin
            // Update status register
            status_reg <= {
                mppt_at_mpp,
                protection_shutdown,
                overvoltage_fault,
                undervoltage_fault,
                overcurrent_fault,
                overtemperature_fault,
                backflow_fault,
                adc_data_valid
            };
            
            // Handle register writes
            if (reg_write) begin
                registers[reg_addr] <= reg_wdata;
                
                // Update configuration based on register writes
                case (reg_addr)
                    8'h00: begin // Control register
                        control_reg <= reg_wdata;
                        mppt_enable <= reg_wdata[0];
                        soft_start_enable <= reg_wdata[1];
                    end
                    // MPPT step size (bytes 2-5)
                    8'h02: mppt_step_size[31:24] <= reg_wdata;
                    8'h03: mppt_step_size[23:16] <= reg_wdata;
                    8'h04: mppt_step_size[15:8] <= reg_wdata;
                    8'h05: mppt_step_size[7:0] <= reg_wdata;
                    // MPPT V_max (bytes 6-9)
                    8'h06: mppt_v_max[31:24] <= reg_wdata;
                    8'h07: mppt_v_max[23:16] <= reg_wdata;
                    8'h08: mppt_v_max[15:8] <= reg_wdata;
                    8'h09: mppt_v_max[7:0] <= reg_wdata;
                    // MPPT V_min (bytes 10-13)
                    8'h0A: mppt_v_min[31:24] <= reg_wdata;
                    8'h0B: mppt_v_min[23:16] <= reg_wdata;
                    8'h0C: mppt_v_min[15:8] <= reg_wdata;
                    8'h0D: mppt_v_min[7:0] <= reg_wdata;
                    default: begin
                        // Other registers handled by register array
                    end
                endcase
            end
            
            // Handle register reads
            if (reg_read) begin
                case (reg_addr)
                    8'h00: registers[8'h00] <= control_reg;
                    8'h01: registers[8'h01] <= status_reg;
                    // V_ref (bytes 14-17)
                    8'h0E: registers[8'h0E] <= mppt_v_ref[31:24];
                    8'h0F: registers[8'h0F] <= mppt_v_ref[23:16];
                    8'h10: registers[8'h10] <= mppt_v_ref[15:8];
                    8'h11: registers[8'h11] <= mppt_v_ref[7:0];
                    // Solar voltage (bytes 18-21)
                    8'h12: registers[8'h12] <= solar_voltage[31:24];
                    8'h13: registers[8'h13] <= solar_voltage[23:16];
                    8'h14: registers[8'h14] <= solar_voltage[15:8];
                    8'h15: registers[8'h15] <= solar_voltage[7:0];
                    // Solar current (bytes 22-25)
                    8'h16: registers[8'h16] <= solar_current[31:24];
                    8'h17: registers[8'h17] <= solar_current[23:16];
                    8'h18: registers[8'h18] <= solar_current[15:8];
                    8'h19: registers[8'h19] <= solar_current[7:0];
                    // Battery voltage (bytes 26-29)
                    8'h1A: registers[8'h1A] <= battery_voltage[31:24];
                    8'h1B: registers[8'h1B] <= battery_voltage[23:16];
                    8'h1C: registers[8'h1C] <= battery_voltage[15:8];
                    8'h1D: registers[8'h1D] <= battery_voltage[7:0];
                    // Battery current (bytes 30-33)
                    8'h1E: registers[8'h1E] <= battery_current[31:24];
                    8'h1F: registers[8'h1F] <= battery_current[23:16];
                    8'h20: registers[8'h20] <= battery_current[15:8];
                    8'h21: registers[8'h21] <= battery_current[7:0];
                    // Temperature 1 (bytes 34-37)
                    8'h22: registers[8'h22] <= temperature_1[31:24];
                    8'h23: registers[8'h23] <= temperature_1[23:16];
                    8'h24: registers[8'h24] <= temperature_1[15:8];
                    8'h25: registers[8'h25] <= temperature_1[7:0];
                    // Temperature 2 (bytes 38-41)
                    8'h26: registers[8'h26] <= temperature_2[31:24];
                    8'h27: registers[8'h27] <= temperature_2[23:16];
                    8'h28: registers[8'h28] <= temperature_2[15:8];
                    8'h29: registers[8'h29] <= temperature_2[7:0];
                    // PWM duty cycle (bytes 42-43)
                    8'h2A: registers[8'h2A] <= {6'b0, pwm_duty_cycle[9:8]};
                    8'h2B: registers[8'h2B] <= pwm_duty_cycle[7:0];
                    default: begin
                        // Return stored register value
                    end
                endcase
            end
        end
    end
    
    // Register read data output
    assign reg_rdata = registers[reg_addr];
    
    // Output assignments
    assign shutdown = protection_shutdown;
    assign fan_drive = protection_fan;
    assign backflow_protection = protection_backflow;

endmodule