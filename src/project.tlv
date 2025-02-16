\m5_TLV_version 1d: tl-x.org
\m5
   use(m5-1.0)
   
   
   // ########################################################
   // #                                                      #
   // #  Empty template for Tiny Tapeout Makerchip Projects  #
   // #                                                      #
   // ########################################################
   
   // ========
   // Settings
   // ========
   
   //-------------------------------------------------------
   // Build Target Configuration
   //
   var(my_design, tt_um_example)   /// The name of your top-level TT module, to match your info.yml.
   var(target, ASIC)   /// Note, the FPGA CI flow will set this to FPGA.
   //-------------------------------------------------------
   
   var(in_fpga, 1)   /// 1 to include the demo board. (Note: Logic will be under /fpga_pins/fpga.)
   var(debounce_inputs, 0)         /// 1: Provide synchronization and debouncing on all input signals.
                                   /// 0: Don't provide synchronization and debouncing.
                                   /// m5_if_defined_as(MAKERCHIP, 1, 0, 1): Debounce unless in Makerchip.
   
   // ======================
   // Computed From Settings
   // ======================
   
   // If debouncing, a user's module is within a wrapper, so it has a different name.
   var(user_module_name, m5_if(m5_debounce_inputs, my_design, m5_my_design))
   var(debounce_cnt, m5_if_defined_as(MAKERCHIP, 1, 8'h03, 8'hff))

\SV
   // Include Tiny Tapeout Lab.
   m4_include_lib(['https:/']['/raw.githubusercontent.com/os-fpga/Virtual-FPGA-Lab/5744600215af09224b7235479be84c30c6e50cb7/tlv_lib/tiny_tapeout_lib.tlv'])
   
module uart_tx 
    #(parameter int FREQUENCY = 10000000, parameter int BAUD_RATE = 9600)
    (
        input logic clk,
        input logic reset,
        input logic tx_dv,
        input logic [7:0] tx_byte, 
        output logic tx_active,
        output logic tx_serial,
        output logic tx_done
    );

    typedef enum logic [2:0] {
        s_IDLE          = 3'b000,
        s_TX_START_BIT  = 3'b001,
        s_TX_DATA_BITS  = 3'b010,
        s_TX_STOP_BIT   = 3'b011,
        s_CLEANUP       = 3'b100
    } state_t;

    localparam int CLKS_PER_BIT = FREQUENCY /  BAUD_RATE;

    state_t r_SM_Main = s_IDLE;
    logic [7:0] r_Clock_Count = 0;
    logic [2:0] r_Bit_Index = 0;
    logic [7:0] r_Tx_Data = 0;
    logic r_Tx_Done = 0;
    logic r_Tx_Active = 0;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            r_SM_Main <= s_IDLE;
            r_Clock_Count <= 0;
            r_Bit_Index <= 0;
            r_Tx_Data <= 0;
            r_Tx_Done <= 0;
            r_Tx_Active <= 0;
            tx_serial <= 1;
        end else begin
            case (r_SM_Main)
                s_IDLE: begin
                    tx_serial <= 1; // Line idle state
                    r_Tx_Done <= 0;
                    r_Clock_Count <= 0;
                    r_Bit_Index <= 0;
                    
                    if (tx_dv) begin
                        r_Tx_Active <= 1;
                        r_Tx_Data <= tx_byte;
                        r_SM_Main <= s_TX_START_BIT;
                    end else begin
                        r_SM_Main <= s_IDLE;
                    end
                end

                s_TX_START_BIT: begin
                    tx_serial <= 0; // Start bit
                    if (r_Clock_Count < CLKS_PER_BIT - 1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                    end else begin
                        r_Clock_Count <= 0;
                        r_SM_Main <= s_TX_DATA_BITS;
                    end
                end

                s_TX_DATA_BITS: begin
                    tx_serial <= r_Tx_Data[r_Bit_Index];
                    if (r_Clock_Count < CLKS_PER_BIT - 1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                    end else begin
                        r_Clock_Count <= 0;
                        if (r_Bit_Index < 7) begin
                            r_Bit_Index <= r_Bit_Index + 1;
                        end else begin
                            r_Bit_Index <= 0;
                            r_SM_Main <= s_TX_STOP_BIT;
                        end
                    end
                end

                s_TX_STOP_BIT: begin
                    tx_serial <= 1; // Stop bit
                    if (r_Clock_Count < CLKS_PER_BIT - 1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                    end else begin
                        r_Tx_Done <= 1;
                        r_Clock_Count <= 0;
                        r_Tx_Active <= 0;
                        r_SM_Main <= s_CLEANUP;
                    end
                end

                s_CLEANUP: begin
                    r_Tx_Done <= 1;
                    r_SM_Main <= s_IDLE;
                end

                default: r_SM_Main <= s_IDLE;
            endcase
        end
    end

    assign tx_active = r_Tx_Active;
    assign tx_done = r_Tx_Done;

endmodule

module uart_rx 
    #(parameter int FREQUENCY = 20_000_000, parameter int BAUD_RATE = 9600)
    (
        input logic clk,
        input logic rx_serial,          // input serial data
        input logic reset,
        output logic rx_done,           // asserts when reception is done
        output logic [7:0] rx_byte      // received byte
    );

    localparam int CLKS_PER_BIT = FREQUENCY / BAUD_RATE;

    typedef enum logic [2:0] {
        s_IDLE          = 3'b000,
        s_RX_START_BIT  = 3'b001,
        s_RX_DATA_BITS  = 3'b010,
        s_RX_STOP_BIT   = 3'b011,
        s_CLEANUP       = 3'b100
    } state_t;

    state_t r_SM_Main = s_IDLE;

    logic r_Rx_Data_R = 1'b1;
    logic r_Rx_Data = 1'b1;

    int unsigned r_Clock_Count = 0;
    int unsigned r_Bit_Index = 0; // 8 bits total
    logic [7:0] r_Rx_Byte = 8'h00;
    logic r_Rx_DV = 1'b0;

    // Purpose: Double-register the incoming data to avoid metastability
    always_ff @(posedge clk) begin
        r_Rx_Data_R <= rx_serial;
        r_Rx_Data   <= r_Rx_Data_R;
    end

    // RX state machine
    always_ff @(posedge clk) begin
        if (reset) begin
            r_SM_Main      <= s_IDLE;
            r_Rx_DV        <= 1'b0;
            r_Clock_Count  <= 0;
            r_Bit_Index    <= 0;
            r_Rx_Byte      <= 8'h00;
        end 
        
        else begin
            case (r_SM_Main)
                s_IDLE: begin
                    r_Rx_DV       <= 1'b0;
                    r_Clock_Count <= 0;
                    r_Bit_Index   <= 0;

                    if (r_Rx_Data == 1'b0) // Start bit detected
                        r_SM_Main <= s_RX_START_BIT;
                end

                s_RX_START_BIT: begin
                    if (r_Clock_Count == (CLKS_PER_BIT - 1) / 2) begin
                        if (r_Rx_Data == 1'b0) begin
                            r_Clock_Count <= 0;  // Reset counter, found the middle
                            r_SM_Main     <= s_RX_DATA_BITS;
                        end else begin
                            r_SM_Main <= s_IDLE;
                        end
                    end else begin
                        r_Clock_Count <= r_Clock_Count + 1;
                    end
                end

                s_RX_DATA_BITS: begin
                    if (r_Clock_Count < CLKS_PER_BIT - 1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                    end else begin
                        r_Clock_Count <= 0;
                        r_Rx_Byte[r_Bit_Index] <= r_Rx_Data;

                        if (r_Bit_Index < 7) begin
                            r_Bit_Index <= r_Bit_Index + 1;
                        end else begin
                            r_Bit_Index <= 0;
                            r_SM_Main   <= s_RX_STOP_BIT;
                        end
                    end
                end

                s_RX_STOP_BIT: begin
                    if (r_Clock_Count < CLKS_PER_BIT - 1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                    end else begin
                        r_Rx_DV       <= 1'b1;
                        r_Clock_Count <= 0;
                        r_SM_Main     <= s_CLEANUP;
                    end
                end

                s_CLEANUP: begin
                    r_SM_Main <= s_IDLE;
                    r_Rx_DV   <= 1'b0;
                end

                default: r_SM_Main <= s_IDLE;
            endcase
        end
    end

    assign rx_done = r_Rx_DV;
    assign rx_byte = r_Rx_Byte;
endmodule


\TLV my_design()

   // following pipe is just an use case of how UART receiver and transmitter controller can be used
   |uart
      @0
         \SV_plus
            uart_rx #(20000000,115200) uart_rx(.clk(*clk),
                                               .reset(*reset),
                                               .rx_serial($rx_serial),
                                               .rx_done($$rx_done),
                                               .rx_byte($$rx_byte[7:0])
                                               );
         $rx_serial = *ui_in[6];   // pmod connector's TxD port
         $received = $rx_done;
         $received_byte[7:0] = $rx_byte[7:0];

      @1
         $tx_dv = $received;
         $tx_byte[7:0] = $received_byte + 8'd1;   // add 1 to the received byte and send the data
         \SV_plus
            uart_tx #(20000000,115200) uart_tx( .clk(*clk),
                                   .reset(*reset),
                                   .tx_dv($tx_dv),
                                   .tx_byte($tx_byte[7:0]),
                                   .tx_active($$tx_active),
                                   .tx_serial($$tx_serial),
                                   .tx_done($$tx_done));
         
         *uo_out[0] = $tx_active;
         *uo_out[1] = $tx_done;
         *uo_out[5] = $tx_serial;   // pmod connector's RxD port
   

   
   
   // Note that pipesignals assigned here can be found under /fpga_pins/fpga.
   
   
   
   
   // Connect Tiny Tapeout outputs. Note that uio_ outputs are not available in the Tiny-Tapeout-3-based FPGA boards.
   //*uo_out = 8'b0;
   m5_if_neq(m5_target, FPGA, ['*uio_out = 8'b0;'])
   m5_if_neq(m5_target, FPGA, ['*uio_oe = 8'b0;'])

// Set up the Tiny Tapeout lab environment.
\TLV tt_lab()
   // Connect Tiny Tapeout I/Os to Virtual FPGA Lab.
   m5+tt_connections()
   // Instantiate the Virtual FPGA Lab.
   m5+board(/top, /fpga, 7, $, , my_design)
   // Label the switch inputs [0..7] (1..8 on the physical switch panel) (top-to-bottom).
   m5+tt_input_labels_viz(['"UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED"'])

\SV

// ================================================
// A simple Makerchip Verilog test bench driving random stimulus.
// Modify the module contents to your needs.
// ================================================

module top(input logic clk, input logic reset, input logic [31:0] cyc_cnt, output logic passed, output logic failed);
   // Tiny tapeout I/O signals.
   logic [7:0] ui_in, uo_out;
   m5_if_neq(m5_target, FPGA, ['logic [7:0] uio_in, uio_out, uio_oe;'])
   logic [31:0] r;  // a random value
   always @(posedge clk) r <= m5_if_defined_as(MAKERCHIP, 1, ['$urandom()'], ['0']);
   assign ui_in = r[7:0];
   m5_if_neq(m5_target, FPGA, ['assign uio_in = 8'b0;'])
   logic ena = 1'b0;
   logic rst_n = ! reset;
   
   /*
   // Or, to provide specific inputs at specific times (as for lab C-TB) ...
   // BE SURE TO COMMENT THE ASSIGNMENT OF INPUTS ABOVE.
   // BE SURE TO DRIVE THESE ON THE B-PHASE OF THE CLOCK (ODD STEPS).
   // Driving on the rising clock edge creates a race with the clock that has unpredictable simulation behavior.
   initial begin
      #1  // Drive inputs on the B-phase.
         ui_in = 8'h0;
      #10 // Step 5 cycles, past reset.
         ui_in = 8'hFF;
      // ...etc.
   end
   */

   // Instantiate the Tiny Tapeout module.
   m5_user_module_name tt(.*);
   
   assign passed = top.cyc_cnt > 80;
   assign failed = 1'b0;
endmodule


// Provide a wrapper module to debounce input signals if requested.
m5_if(m5_debounce_inputs, ['m5_tt_top(m5_my_design)'])
\SV



// =======================
// The Tiny Tapeout module
// =======================

module m5_user_module_name (
    input  wire [7:0] ui_in,    // Dedicated inputs - connected to the input switches
    output wire [7:0] uo_out,   // Dedicated outputs - connected to the 7 segment display
    m5_if_eq(m5_target, FPGA, ['/']['*'])   // The FPGA is based on TinyTapeout 3 which has no bidirectional I/Os (vs. TT6 for the ASIC).
    input  wire [7:0] uio_in,   // IOs: Bidirectional Input path
    output wire [7:0] uio_out,  // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,   // IOs: Bidirectional Enable path (active high: 0=input, 1=output)
    m5_if_eq(m5_target, FPGA, ['*']['/'])
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
   wire reset = ! rst_n;

   // List all potentially-unused inputs to prevent warnings
   wire _unused = &{ena, clk, rst_n, 1'b0};

\TLV
   /* verilator lint_off UNOPTFLAT */
   m5_if(m5_in_fpga, ['m5+tt_lab()'], ['m5+my_design()'])

\SV_plus
   
   // ==========================================
   // If you are using Verilog for your design,
   // your Verilog logic goes here.
   // Note, output assignments are in my_design.
   // ==========================================

\SV
endmodule
