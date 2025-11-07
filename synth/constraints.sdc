# SDC constraints for MPPT ASIC controller
# Clock definitions and timing constraints

# Main system clock (50 MHz)
create_clock -name clk -period 20.0 [get_ports clk]

# Clock uncertainty (jitter, skew)
set_clock_uncertainty 0.5 [get_clocks clk]

# Input delays (relative to clock)
set_input_delay -clock clk 2.0 [get_ports rst_n]
set_input_delay -clock clk 2.0 [get_ports {battery_voltage_sense[*]}]
set_input_delay -clock clk 2.0 [get_ports {battery_current_sense[*]}]
set_input_delay -clock clk 2.0 [get_ports {solar_voltage_sense[*]}]
set_input_delay -clock clk 2.0 [get_ports {solar_current_sense[*]}]
set_input_delay -clock clk 2.0 [get_ports {temperature_sense_1[*]}]
set_input_delay -clock clk 2.0 [get_ports {temperature_sense_2[*]}]
set_input_delay -clock clk 2.0 [get_ports uart_rx]
set_input_delay -clock clk 2.0 [get_ports {i2c_sda}]
set_input_delay -clock clk 2.0 [get_ports i2c_scl]
set_input_delay -clock clk 2.0 [get_ports spi_sck]
set_input_delay -clock clk 2.0 [get_ports spi_mosi]
set_input_delay -clock clk 2.0 [get_ports spi_ss]

# Output delays (relative to clock)
set_output_delay -clock clk 2.0 [get_ports shutdown]
set_output_delay -clock clk 2.0 [get_ports fan_drive]
set_output_delay -clock clk 2.0 [get_ports backflow_protection]
set_output_delay -clock clk 2.0 [get_ports pwm_high]
set_output_delay -clock clk 2.0 [get_ports pwm_low]
set_output_delay -clock clk 2.0 [get_ports uart_tx]
set_output_delay -clock clk 2.0 [get_ports {spi_miso}]

# Bidirectional ports
set_input_delay -clock clk 2.0 [get_ports {i2c_sda}]
set_output_delay -clock clk 2.0 [get_ports {i2c_sda}]

# False paths for asynchronous resets and status signals
set_false_path -from [get_ports rst_n] -to [all_registers]

# Multi-cycle paths for slow interfaces (UART, I2C, SPI)
# UART at 9600 baud - much slower than 50MHz clock
set_multicycle_path -from [get_clocks clk] -to [get_ports uart_tx] 500
set_multicycle_path -from [get_ports uart_rx] -to [get_clocks clk] 500

# I2C interface timing (assume 100kHz I2C clock)
set_multicycle_path -from [get_clocks clk] -to [get_ports {i2c_sda}] 500
set_multicycle_path -from [get_ports {i2c_sda}] -to [get_clocks clk] 500
set_multicycle_path -from [get_ports i2c_scl] -to [get_clocks clk] 500

# SPI interface timing (assume up to 10MHz SPI clock)
set_multicycle_path -from [get_clocks clk] -to [get_ports spi_miso] 5
set_multicycle_path -from [get_ports {spi_sck spi_mosi spi_ss}] -to [get_clocks clk] 5

# Don't optimize across asynchronous boundaries
set_false_path -from [get_ports {uart_rx i2c_scl spi_sck spi_ss}] -to [all_registers -clock clk]

# Maximum fanout constraints
set_max_fanout 20 [all_nets]

# Maximum transition time
set_max_transition 1.0 [all_nets]

# Operating conditions (generic - adjust for target process)
# set_operating_conditions -library <library> -max <corner> -min <corner>