# asic-mppt

ASIC implementation of a Maximum Power Point Tracking (MPPT) controller

## Project Structure

```
asic-mppt/
├── LICENSE                   # Project license
├── Makefile                  # Build automation
├── README.md                 # This file
├── run.sh                    # Main build script
├── rtl/                      # Verilog RTL source files
│   ├── adc_interface.v       # ADC interface module
│   ├── fixed_point_math.v    # Fixed-point math utilities
│   ├── i2c_slave.v           # I2C slave interface
│   ├── mppt.v                # Main MPPT controller module
│   ├── mppt_ic_core.v        # MPPT incremental conductance core
│   ├── protection_logic.v    # Protection and safety logic
│   ├── pwm_generator.v       # PWM signal generator
│   ├── spi_slave.v           # SPI slave interface
│   └── uart_modbus.v         # UART Modbus interface
└── sim/                      # Simulation files
    └── tb_mppt_controller.v  # Testbench for MPPT controller
```

## Setup Instructions

### Prerequisites

Install the following tools:

```bash
# macOS (using Homebrew)
brew install verilator icarus-verilog yosys openroad mkdocs

# Ubuntu/Debian
sudo apt-get install verilator iverilog yosys openroad mkdocs python3-pip
pip3 install mkdocs

# Note: Magic and Netgen require PDK setup for verification
```

### Installation

1. Clone or download the project
2. Make the build script executable:
   ```bash
   chmod +x run.sh
   ```

## Build Commands

The `run.sh` script provides automated build flow with the following targets:

| Command          | Description                                          |
|------------------|------------------------------------------------------|
| `./run.sh help`  | Show available commands and usage                    |
| `./run.sh lint`  | Run Verilator linting on RTL files                   |
| `./run.sh sim`   | Run simulation with Icarus Verilog                   |
| `./run.sh synth` | Run synthesis with Yosys                             |
| `./run.sh pnr`   | Run place and route with OpenROAD                    |
| `./run.sh verify`| Run DRC/LVS verification                             |
| `./run.sh docs`  | Build documentation with MkDocs                      |
| `./run.sh clean` | Clean all generated files                            |
| `./run.sh test`  | Quick test (lint + sim only)                         |
| `./run.sh asic`  | Full ASIC flow (lint → sim → synth → pnr → verify)   |
| `./run.sh all`   | Complete build flow (lint + sim + synth + docs)      |
