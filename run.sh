#!/bin/bash

# asic-mppt Build Script
# Automated build flow from RTL to GDSII

# Project configuration
PROJECT="asic-mppt"
TOP_MODULE="mppt_controller"
TESTBENCH="tb_mppt_controller"

# Directories
RTL_DIR="rtl"
SIM_DIR="sim"
SYNTH_DIR="synth"
PNR_DIR="pnr"
DOCS_DIR="docs"
OUTPUT_DIR="output"
REPORTS_DIR="$SYNTH_DIR/reports"

# Tools
VERILATOR="verilator"
IVERILOG="iverilog"
VVP="vvp"
YOSYS="yosys"
OPENROAD="openroad"
MAGIC="magic"
NETGEN="netgen"
MKDOCS="mkdocs"
DOXYGEN="doxygen"

# Source files
RTL_SOURCES=$(ls $RTL_DIR/*.v 2>/dev/null)
TB_SOURCES=$(ls $SIM_DIR/*.v 2>/dev/null)

# Verilator flags
VERILATOR_FLAGS="--lint-only -Wall -Wno-fatal"

# Icarus Verilog flags
IVERILOG_FLAGS="-g2012 -Wall"

# Output files
SYNTH_OUTPUT="$REPORTS_DIR/${TOP_MODULE}_synth.v"
SIM_OUTPUT="$SIM_DIR/${TESTBENCH}.vvp"
WAVE_OUTPUT="$SIM_DIR/waveforms/${TESTBENCH}.vcd"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print usage
usage() {
    echo -e "${BLUE}asic-mppt Build System${NC}"
    echo ""
    echo "Usage: $0 [target]"
    echo ""
    echo "Available targets:"
    echo "  all      - Run complete flow (lint + sim + synth + docs)"
    echo "  lint     - Run Verilator linting"
    echo "  sim      - Run simulation with Icarus Verilog"
    echo "  synth    - Run synthesis with Yosys"
    echo "  pnr      - Run place and route with OpenROAD"
    echo "  verify   - Run DRC/LVS verification"
    echo "  docs     - Build documentation"
    echo "  clean    - Clean all generated files"
    echo "  test     - Quick test (lint + sim only)"
    echo "  asic     - Full ASIC flow"
    echo "  help     - Show this help message"
    echo ""
}

# Lint RTL with Verilator
lint() {
    echo -e "${BLUE}Running Verilator lint...${NC}"
    mkdir -p $REPORTS_DIR
    for file in $RTL_SOURCES; do
        echo "  Linting $file..."
        $VERILATOR $VERILATOR_FLAGS $file 2>&1 | tee -a $REPORTS_DIR/lint.log
    done
    echo -e "${GREEN}✓ Linting completed${NC}"
}

# Simulate with Icarus Verilog
sim() {
    echo -e "${BLUE}Running simulation...${NC}"
    mkdir -p $SIM_DIR/waveforms
    if [ ! -f "$SIM_OUTPUT" ]; then
        compile_tb
    fi
    $VVP $SIM_OUTPUT
    echo -e "${GREEN}✓ Simulation completed${NC}"
    echo -e "${YELLOW}Waveform saved to: $WAVE_OUTPUT${NC}"
}

compile_tb() {
    echo -e "${BLUE}Compiling testbench...${NC}"
    $IVERILOG $IVERILOG_FLAGS -o $SIM_OUTPUT $RTL_SOURCES $TB_SOURCES
}

# Synthesize with Yosys
synth() {
    echo -e "${BLUE}Running synthesis...${NC}"
    mkdir -p $REPORTS_DIR
    cd $SYNTH_DIR
    $YOSYS -s synth.ys 2>&1 | tee reports/synthesis.log
    cd ..
    echo -e "${GREEN}✓ Synthesis completed${NC}"
    echo -e "${YELLOW}Reports available in: $REPORTS_DIR${NC}"
}

# Place and Route with OpenROAD
pnr() {
    synth
    echo -e "${BLUE}Running place and route...${NC}"
    mkdir -p $PNR_DIR/results
    cd $PNR_DIR
    $OPENROAD -exit floorplan.tcl 2>&1 | tee results/pnr.log
    cd ..
    echo -e "${GREEN}✓ Place and route completed${NC}"
}

# DRC/LVS Verification
verify() {
    pnr
    echo -e "${BLUE}Running DRC/LVS verification...${NC}"
    mkdir -p $OUTPUT_DIR
    echo -e "${YELLOW}Note: DRC/LVS requires Magic and Netgen with PDK setup${NC}"
    echo -e "${GREEN}✓ Verification stage ready${NC}"
}

# Build documentation
docs() {
    echo -e "${BLUE}Building documentation...${NC}"
    if [ -f "$DOCS_DIR/mkdocs.yml" ]; then
        cd $DOCS_DIR
        $MKDOCS build
        cd ..
        echo -e "${GREEN}✓ Documentation built successfully${NC}"
    else
        echo -e "${YELLOW}⚠ MkDocs configuration not found, creating basic structure...${NC}"
        mkdir -p $DOCS_DIR
        echo -e "${GREEN}✓ Documentation directory created${NC}"
    fi
}

# Clean generated files
clean() {
    echo -e "${BLUE}Cleaning generated files...${NC}"
    rm -rf $SIM_DIR/*.vvp
    rm -rf $SIM_DIR/waveforms/*.vcd
    rm -rf $REPORTS_DIR/*
    rm -rf $PNR_DIR/results/*
    rm -rf $OUTPUT_DIR/*
    rm -rf $DOCS_DIR/site
    echo -e "${GREEN}✓ Clean completed${NC}"
}

# Quick test (lint + sim only)
test() {
    lint
    sim
    echo -e "${GREEN}✓ Quick test completed${NC}"
}

# Full ASIC flow
asic() {
    lint
    sim
    synth
    pnr
    verify
    echo -e "${GREEN}✓ Full ASIC flow completed${NC}"
}

# Main script logic
case "$1" in
    all)
        lint
        sim
        synth
        docs
        echo -e "${GREEN}✓ Complete build flow finished successfully${NC}"
        ;;
    lint)
        lint
        ;;
    sim)
        sim
        ;;
    synth)
        synth
        ;;
    pnr)
        pnr
        ;;
    verify)
        verify
        ;;
    docs)
        docs
        ;;
    clean)
        clean
        ;;
    test)
        test
        ;;
    asic)
        asic
        ;;
    help|*)
        usage
        ;;
esac