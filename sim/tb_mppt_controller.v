`timescale 1ns / 1ps

//==============================================================================
// Testbench for MPPT Controller
//==============================================================================
// This testbench verifies the complete MPPT controller functionality including:
//   - ADC interface and signal conditioning
//   - Incremental Conductance MPPT algorithm
//   - Protection logic
//   - PWM generation
//   - Communication interfaces (I2C, SPI, UART)
//==============================================================================

module tb_mppt_controller;

    // Clock and reset
    reg clk;
    reg rst_n;
    
    // ADC inputs (12-bit)
    reg [11:0] battery_voltage_sense;
    reg [11:0] battery_current_sense;
    reg [11:0] solar_voltage_sense;
    reg [11:0] solar_current_sense;
    reg [11:0] temperature_sense_1;
    reg [11:0] temperature_sense_2;
    
    // Digital outputs
    wire shutdown;
    wire fan_drive;
    wire backflow_protection;
    wire pwm_high;
    wire pwm_low;
    
    // UART interface
    reg uart_rx;
    wire uart_tx;
    
    // I2C interface
    wire i2c_sda;
    reg i2c_scl;
    reg i2c_sda_drive;
    reg i2c_sda_out;
    
    // SPI interface
    reg spi_sck;
    reg spi_mosi;
    wire spi_miso;
    reg spi_ss;
    
    // I2C tri-state control
    assign i2c_sda = i2c_sda_drive ? i2c_sda_out : 1'bz;
    
    // Clock generation (50 MHz)
    parameter CLK_PERIOD = 20; // 20ns = 50MHz
    always #(CLK_PERIOD/2) clk = ~clk;
    
    // DUT instantiation
    mppt_controller #(
        .CLK_FREQ(50_000_000),
        .DATA_WIDTH(32),
        .I2C_ADDR(7'h50),
        .MODBUS_ADDR(8'h01)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .battery_voltage_sense(battery_voltage_sense),
        .battery_current_sense(battery_current_sense),
        .solar_voltage_sense(solar_voltage_sense),
        .solar_current_sense(solar_current_sense),
        .temperature_sense_1(temperature_sense_1),
        .temperature_sense_2(temperature_sense_2),
        .shutdown(shutdown),
        .fan_drive(fan_drive),
        .backflow_protection(backflow_protection),
        .pwm_high(pwm_high),
        .pwm_low(pwm_low),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .i2c_sda(i2c_sda),
        .i2c_scl(i2c_scl),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_ss(spi_ss)
    );
    
    // Test variables
    integer i;
    reg [7:0] test_data;
    
    // ADC value conversion helpers
    // Battery voltage: 0-60V -> 0-4095
    // Solar voltage: 0-60V -> 0-4095
    // Current: 0-20A -> 0-4095
    // Temperature: 0-150°C -> 0-4095
    
    function [11:0] voltage_to_adc;
        input real voltage; // in Volts
        begin
            voltage_to_adc = (voltage / 60.0) * 4095.0;
        end
    endfunction
    
    function [11:0] current_to_adc;
        input real current; // in Amperes
        begin
            current_to_adc = (current / 20.0) * 4095.0;
        end
    endfunction
    
    function [11:0] temp_to_adc;
        input real temperature; // in Celsius
        begin
            temp_to_adc = (temperature / 150.0) * 4095.0;
        end
    endfunction
    
    // I2C tasks
    task i2c_start;
        begin
            i2c_sda_drive = 1;
            i2c_sda_out = 1;
            i2c_scl = 1;
            #1000;
            i2c_sda_out = 0;
            #1000;
            i2c_scl = 0;
            #1000;
        end
    endtask
    
    task i2c_stop;
        begin
            i2c_sda_drive = 1;
            i2c_sda_out = 0;
            i2c_scl = 0;
            #1000;
            i2c_scl = 1;
            #1000;
            i2c_sda_out = 1;
            #1000;
        end
    endtask
    
    task i2c_write_byte;
        input [7:0] data;
        integer j;
        begin
            i2c_sda_drive = 1;
            for (j = 7; j >= 0; j = j - 1) begin
                i2c_sda_out = data[j];
                #1000;
                i2c_scl = 1;
                #1000;
                i2c_scl = 0;
                #1000;
            end
            // ACK
            i2c_sda_drive = 0;
            #1000;
            i2c_scl = 1;
            #1000;
            i2c_scl = 0;
            #1000;
        end
    endtask
    
    // SPI tasks
    task spi_transfer;
        input [7:0] data_out;
        output [7:0] data_in;
        integer j;
        begin
            spi_ss = 0;
            #100;
            for (j = 7; j >= 0; j = j - 1) begin
                spi_mosi = data_out[j];
                #100;
                spi_sck = 1;
                #100;
                data_in[j] = spi_miso;
                spi_sck = 0;
                #100;
            end
            spi_ss = 1;
            #100;
        end
    endtask
    
    // Simulation scenarios
    task test_normal_operation;
        begin
            $display("\n=== Test 1: Normal Operation ===");
            
            // Set normal operating conditions
            // Solar: 25V, 5A (125W)
            solar_voltage_sense = voltage_to_adc(25.0);
            solar_current_sense = current_to_adc(5.0);
            
            // Battery: 48V, 2A
            battery_voltage_sense = voltage_to_adc(48.0);
            battery_current_sense = current_to_adc(2.0);
            
            // Temperature: 35°C
            temperature_sense_1 = temp_to_adc(35.0);
            temperature_sense_2 = temp_to_adc(35.0);
            
            // Enable MPPT via I2C
            $display("Enabling MPPT via I2C...");
            i2c_start();
            i2c_write_byte({7'h50, 1'b0}); // Address + Write
            i2c_write_byte(8'h00);          // Control register
            i2c_write_byte(8'h01);          // Enable MPPT
            i2c_stop();
            
            // Wait for MPPT to stabilize
            #1_000_000; // 1ms
            
            $display("PWM High: %b, PWM Low: %b", pwm_high, pwm_low);
            $display("Shutdown: %b, Fan: %b, Backflow: %b", 
                     shutdown, fan_drive, backflow_protection);
        end
    endtask
    
    task test_overvoltage_protection;
        begin
            $display("\n=== Test 2: Overvoltage Protection ===");
            
            // Set overvoltage condition
            solar_voltage_sense = voltage_to_adc(65.0); // Above 60V limit
            solar_current_sense = current_to_adc(5.0);
            battery_voltage_sense = voltage_to_adc(48.0);
            battery_current_sense = current_to_adc(2.0);
            temperature_sense_1 = temp_to_adc(35.0);
            temperature_sense_2 = temp_to_adc(35.0);
            
            #500_000; // Wait for protection to trigger
            
            if (shutdown) begin
                $display("PASS: Overvoltage protection triggered");
            end else begin
                $display("FAIL: Overvoltage protection did not trigger");
            end
        end
    endtask
    
    task test_overtemperature_protection;
        begin
            $display("\n=== Test 3: Overtemperature Protection ===");
            
            // Normal voltage/current
            solar_voltage_sense = voltage_to_adc(25.0);
            solar_current_sense = current_to_adc(5.0);
            battery_voltage_sense = voltage_to_adc(48.0);
            battery_current_sense = current_to_adc(2.0);
            
            // High temperature
            temperature_sense_1 = temp_to_adc(85.0); // Above 80°C limit
            temperature_sense_2 = temp_to_adc(85.0);
            
            #500_000; // Wait for protection to trigger
            
            if (shutdown) begin
                $display("PASS: Overtemperature protection triggered");
            end else begin
                $display("FAIL: Overtemperature protection did not trigger");
            end
            
            if (fan_drive) begin
                $display("PASS: Fan activated");
            end else begin
                $display("FAIL: Fan not activated");
            end
        end
    endtask
    
    task test_spi_readout;
        reg [7:0] spi_data;
        begin
            $display("\n=== Test 4: SPI Data Readout ===");
            
            // Reset to normal conditions
            solar_voltage_sense = voltage_to_adc(25.0);
            solar_current_sense = current_to_adc(5.0);
            battery_voltage_sense = voltage_to_adc(48.0);
            battery_current_sense = current_to_adc(2.0);
            temperature_sense_1 = temp_to_adc(35.0);
            temperature_sense_2 = temp_to_adc(35.0);
            
            #100_000;
            
            // Read status register via SPI
            $display("Reading status register via SPI...");
            spi_transfer(8'h01, spi_data); // Address
            spi_transfer(8'h00, spi_data); // Read data
            $display("Status register: 0x%02h", spi_data);
        end
    endtask
    
    task test_mppt_tracking;
        real solar_v;
        begin
            $display("\n=== Test 5: MPPT Tracking ===");
            
            // Simulate changing solar conditions
            for (solar_v = 20.0; solar_v <= 30.0; solar_v = solar_v + 1.0) begin
                solar_voltage_sense = voltage_to_adc(solar_v);
                solar_current_sense = current_to_adc(5.0);
                battery_voltage_sense = voltage_to_adc(48.0);
                battery_current_sense = current_to_adc(2.0);
                temperature_sense_1 = temp_to_adc(35.0);
                temperature_sense_2 = temp_to_adc(35.0);
                
                #200_000; // Wait for MPPT to adjust
                
                $display("Solar V: %.1fV, PWM: %b", solar_v, pwm_high);
            end
        end
    endtask
    
    // Main test sequence
    initial begin
        // Initialize signals
        clk = 0;
        rst_n = 0;
        uart_rx = 1;
        i2c_scl = 1;
        i2c_sda_drive = 0;
        i2c_sda_out = 1;
        spi_sck = 0;
        spi_mosi = 0;
        spi_ss = 1;
        
        // Initialize ADC inputs to safe values
        battery_voltage_sense = voltage_to_adc(48.0);
        battery_current_sense = current_to_adc(0.0);
        solar_voltage_sense = voltage_to_adc(25.0);
        solar_current_sense = current_to_adc(0.0);
        temperature_sense_1 = temp_to_adc(25.0);
        temperature_sense_2 = temp_to_adc(25.0);
        
        // VCD dump for waveform viewing
        $dumpfile("sim/waveforms/tb_mppt_controller.vcd");
        $dumpvars(0, tb_mppt_controller);
        
        // Reset sequence
        $display("=== MPPT Controller Testbench ===");
        $display("Starting reset sequence...");
        #100;
        rst_n = 1;
        #1000;
        
        // Run tests
        test_normal_operation();
        #2_000_000;
        
        test_overvoltage_protection();
        #2_000_000;
        
        // Reset for next test
        rst_n = 0;
        #100;
        rst_n = 1;
        #1000;
        
        test_overtemperature_protection();
        #2_000_000;
        
        // Reset for next test
        rst_n = 0;
        #100;
        rst_n = 1;
        #1000;
        
        test_spi_readout();
        #1_000_000;
        
        test_mppt_tracking();
        #2_000_000;
        
        $display("\n=== All Tests Complete ===");
        $finish;
    end
    
    // Timeout watchdog
    initial begin
        #50_000_000; // 50ms timeout
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule