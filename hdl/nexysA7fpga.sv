/*
 * ===========================================================================
 * File:    nexysA7fpga.sv
 * Summary: Top-Level SystemVerilog Module for Nexys A7 FPGA
 * ===========================================================================
 * Purpose: This module defines the top-level interface for the Nexys A7 FPGA,
 *          integrating a Microblaze-based embedded system (embsys) with external
 *          peripherals. It connects switches, buttons, LEDs, 7-segment displays,
 *          PWM outputs for environmental control, I2C for sensors/display, and
 *          UART for communication.
 *
 * Course:  ECE 544 - Embedded Systems Design, Winter 2025
 * Project: Final Project - Environmental Control System
 * Date:    March 20, 2025
 *
 *
 * Dependencies:
 *   - embsys: Embedded system IP block (Microblaze, AXI peripherals)
 *   - Vivado: Synthesis and implementation tool for port mapping
 *
 * Notes:
 *   - Clock input is 100 MHz (clk).
 *   - GPIO inputs are mapped to buttons and switches, with outputs to LEDs and PWM.
 *   - I2C uses tri-state IO for communication with BME280, TSL2561, and OLED.
 *   - UART connects to USB for debugging/logging.
 */

`timescale 1 ps / 1 ps  // Timescale: 1 ps precision, 1 ps resolution

/* ===========================================================================
 * Module Definition
 * ===========================================================================
 */
module nexysA7fpga (
    // Clock and Reset Inputs
    input logic clk,         // 100 MHz system clock input
    input logic btnCpuReset, // CPU reset button (active low)

    // Switch and Button Inputs
    input logic [15:0] sw,    // 16 slide switches
    input logic        btnU,  // Up button
    input logic        btnR,  // Right button
    input logic        btnL,  // Left button
    input logic        btnD,  // Down button
    input logic        btnC,  // Center button

    // LED and Display Outputs
    output logic [15:0] led,  // 16 LEDs for status/indicators
    output logic [ 7:0] an,   // 8 anode signals for 7-segment display
    output logic [ 6:0] seg,  // 7 cathode segments for 7-segment display
    output logic        dp,   // Decimal point for 7-segment display

    // PWM Outputs for Environmental Control (via PMOD JA)
    output logic pid_RED1,  // Red LED status (PMOD JA1)
    output logic pid_GRN1,  // Green LED status (PMOD JA2)
    output logic pid_BLU1,  // Blue LED status (PMOD JA3)
    output logic pid_HEAT,  // Heater control (PMOD JA4)
    output logic pid_FAN,   // Fan control (PMOD JA7)
    output logic pid_LIGHT, // Light control (PMOD JA8)

    // I2C Interface (Tri-State)
    inout logic sclk_io,  // I2C clock line (SCL)
    inout logic sda_io,   // I2C data line (SDA)

    // UART Interface
    input  logic usb_uart_rxd,  // UART receive data from USB
    output logic usb_uart_txd   // UART transmit data to USB
);

  /* ===========================================================================
 * Internal Signal Declarations
 * ===========================================================================
 */
  logic [15:0] gpio_btn;  // Concatenated button inputs for GPIO
  logic [15:0] gpio_sw;  // Switch inputs mapped to GPIO

  /* ===========================================================================
 * Signal Assignments
 * ===========================================================================
 */
  // Map buttons to 16-bit GPIO input (5 buttons, upper 11 bits zero)
  assign gpio_btn = {11'h000, btnC, btnU, btnD, btnL, btnR};
  // Map switches directly to 16-bit GPIO input
  assign gpio_sw  = sw;

  /* ===========================================================================
 * Module Instantiation
 * ===========================================================================
 */
  embsys embsys_i (
      // 7-Segment Display Outputs
      .an_0 (an),  // Anode signals for 8-digit display
      .dp_0 (dp),  // Decimal point output
      .seg_0(seg), // Segment outputs (a-g)

      // Button Inputs (Direct Connections)
      .btnC_0(btnC),  // Center button
      .btnD_0(btnD),  // Down button
      .btnL_0(btnL),  // Left button
      .btnR_0(btnR),  // Right button
      .btnU_0(btnU),  // Up button

      // Clock and Reset
      .clk_100MHz(clk),         // 100 MHz clock input
      .resetn    (btnCpuReset), // Active-low reset from CPU reset button

      // GPIO Inputs
      .gpio_btn_i_tri_i(gpio_btn),  // 16-bit button input vector
      .gpio_sw_i_tri_i (gpio_sw),   // 16-bit switch input vector

      // LED Outputs
      .led_0(led),  // 16-bit LED output vector

      // PWM Outputs (Environmental Control)
      .pid_BLU1 (pid_BLU1),   // Blue LED status output
      .pid_FAN  (pid_FAN),    // Fan control output
      .pid_GRN1 (pid_GRN1),   // Green LED status output
      .pid_HEAT (pid_HEAT),   // Heater control output
      .pid_LIGHT(pid_LIGHT),  // Light control output
      .pid_RED1 (pid_RED1),   // Red LED status output

      // I2C Interface
      .sclk_io(sclk_io),  // I2C clock (tri-state)
      .sda_io (sda_io),   // I2C data (tri-state)

      // UART Interface
      .uart_rtl_0_rxd(usb_uart_rxd),  // UART receive input
      .uart_rtl_0_txd(usb_uart_txd),  // UART transmit output

      // Switch Inputs (Redundant Direct Connection)
      .sw_0(sw)  // 16-bit switch input (also via gpio_sw)
  );

endmodule
