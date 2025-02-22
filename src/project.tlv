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
\TLV imem(@_stage)
   // Instruction Memory containing program.
   @_stage
      \SV_plus
         // The program in an instruction memory.
         reg [7:0] instrs [15:0], datam[15:0];
         initial begin
             instrs[0] = 8'h70; // Custom 8-bit data for instruction 0
             instrs[1] = 8'h01; // Custom 8-bit data for instruction 1
             instrs[2] = 8'h80; // Custom 8-bit data for instruction 2
             instrs[3] = 8'h72;
             instrs[4] = 8'h13;
             instrs[5] = 8'h82;
             instrs[6] = 8'hC7;
             instrs[7] = 8'h35;
             instrs[8] = 8'hFF;
             instrs[9] = 8'hFF; // Custom data for instruction 10
             ///data values
             datam[0] =8'h00;
             datam[1] =8'h06;
             datam[2] =8'h04;
             datam[3] =8'h01;
             datam[4] =8'h09;
             datam[8] =8'h05;
         end
      /* verilator lint_off WIDTHEXPAND */
      $instr_mem[7:0] = instrs\[$imem_rd_addr[3:0]\];
      ?$rd_en
         $data_rd[7:0] = datam\[$idata_rd_addr[3:0]\];
      \SV_plus
         always@(posedge clk)
            if($wr_en)
               datam\[$idata_wr_addr[3:0]\] <= $data_wr[7:0];
         always@(posedge clk)
            if($instr_wr_en)
               instrs\[$imem_wr_addr[3:0]\] <= $instr_wr[7:0];
      /* verilator lint_off WIDTHEXPAND */

\SV


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

//`timescale  1ns/1ps

//---------------------------------------------------------controller---------------------------------------------------

module Controller (
       input clk,
       input reset,
       input start,
       input low,
       input med,
       input hig,
       output heating_op,
       output spinning_op, 
       output pouring_op,
       output waiting_op,
       output prog_op,
       output level_op,
       output temp_op,
       output dura_op
   );

   /*
   reset is also used as stop here 
   */

   //-------------------------------------------------------state variable-------------------------------------------------

       reg [2:0] state_var;                //holds the state values
       reg [2:0] next_state_var;           //holds the next state values 
       reg [2:0] prev_state;
       always @(next_state_var) begin
           state_var <= next_state_var;    //transfers the state values for the next state
       end
      
   //--------------------------------------------------------output variables----------------------------------------------
      
      reg heating;
      reg pouring;
      reg waiting;
      reg spinning;
      reg prog_op_reg;
      reg level_op_reg;
      reg temp_op_reg;
      reg dura_op_reg;
      
   //--------------------------------------------------------state values--------------------------------------------------

       //these variables holds the value of the state variable for comparison

       parameter program_selection = 3'd1;             
       parameter temperature_selection = 3'd2;
       parameter water_level_selection = 3'd3;
       parameter duration = 3'd4;
       parameter wash = 3'd5;
       parameter rinse = 3'd6;
       parameter dry = 3'd7;

   //--------------------------------------------------------parameters---------------------------------------------------

       parameter med_heating = 12'd420; //12'd35; //enter med heating time => depends on temp     => 2.5 mins
       parameter hig_heating = 12'd840; //12'd70; //enter hig heating time => depends on temp     => 5 mins
       parameter low_pouring = 12'd200; //12'd28; //enter low pouring time => depends on level    => 2 mins
       parameter med_pouring = 12'd350; //12'd42; //enter med pouring time => depends on level    => 3 mins
       parameter hig_pouring = 12'd400; //12'd56; //enter hig pouring time => depends on level    => 4 mins
       /*
           proper time assignment is left that should be done here 
           low timer => 5 mins 
           med timer => 15 mins 
           hig timer => 30 mins
       */
       parameter low_timer = 12'd2000; //12'd70; //enter time for low duration wash => depends on timer
       parameter med_timer = 12'd3000; //12'd210; //enter time for low duration wash => depends on timer
       parameter hig_timer = 12'd4000; //12'd420; //enter time for low duration wash => depends on timer


   //-------------------------------------------------------input_for_states-----------------------------------------------

       reg [1:0] timer;          //stores time signal

       reg [1:0] Program;          //stores mode signal

       reg [1:0] temp;             //stores temperature signal

       reg [1:0] level;            //stores water level signal

       reg [1:0] counter_status;   //stores counter status

       //reg count_done;             //doing nothing important 

       reg rinse_status;           //provide rinse status for counter

       reg dry_status;             //provide drying status for counter

       reg [11:0] max_count;

   //----------------------------------------------------------controller--------------------------------------------------

       //always @(posedge clk or posedge reset or posedge start) begin
       always @(posedge clk) begin
           prev_state <= state_var;
           //synthesizable INITIAL BLOCK
           if(reset)
               //when reset is high the value of all the registers changes to zero (DEFAULT VALUE)
               begin
                   timer <= 2'b0;
                   Program <= 2'b0;
                   temp <= 2'b0;
                   level <= 2'b0;
                   prog_op_reg <= 1'b0;
                   dura_op_reg <= 1'b0;
                   level_op_reg <= 1'b0;
                   temp_op_reg <= 1'b0;
                   //counter_status <= 2'b0;
                   next_state_var <= 3'b0;
                   //count_done <= 1'b0;
                   //rinse_status <= 1'b0;
                   //dry_status <= 1'b0;
                   //max_count <= 12'b0;

                   //assigning output vairables to zero during reset

                   //heating <= 1'b0;
                   waiting <= 1'b0;
                   //spinning <= 1'b0;
                   //pouring <= 1'b0;
               end
           else if(start)
               //when start signal is applied the value of the state variable increases by one
               begin
                   next_state_var <= 3'b01;
               end
           else 
               begin
                   case (state_var)   

                       //Program selection stage

                       program_selection:
                           begin 
                               prog_op_reg <= 1'b1;
                               waiting <= 1'b1;
                               if(low)
                                   begin
                                       Program <= 2'd1;
                                   end
                               else if(med)
                                   begin
                                       Program <= 2'd2;
                                   end
                               else if(hig)
                                   begin
                                       Program <= 2'd3;
                                   end
                               else if(next_state_var == temperature_selection |(Program != 2'b0 && (!low & !med & !hig)))
                                   begin
                                       next_state_var <= temperature_selection;
                                       waiting <= 1'b0;
                                   end
                               else
                                   next_state_var <= program_selection;
                           end

                       //temperature selection stage 

                       temperature_selection:
                           begin
                               temp_op_reg <= 1'b1;
                               waiting <= 1'b1;
                               if(low)
                                   begin
                                       temp <= 2'd1;
                                   end
                               else if(med)
                                   begin
                                       temp <= 2'd2;
                                   end
                               else if(hig)
                                   begin
                                       temp <= 2'd3;
                                   end
                               else if(next_state_var == water_level_selection |(temp != 2'b0 && (!low & !med & !hig)))
                                   begin
                                       next_state_var <= water_level_selection;
                                       waiting <= 1'b0;
                                   end
                               else 
                                   next_state_var <= temperature_selection;
                           end

                       //water level selection stage 

                       water_level_selection:
                           begin
                               level_op_reg <= 1'b1;
                               waiting <= 1'b1;
                               if(low)
                                   begin
                                       level <= 2'd1;
                                   end
                               else if(med)
                                   begin
                                       level <= 2'd2;
                                   end
                               else if(hig)
                                   begin
                                       level <= 2'd3;
                                   end
                               else if(next_state_var == duration |(level != 2'b0 && (!low & !med & !hig)))
                                   begin
                                       next_state_var <= duration;
                                       waiting <= 1'b0;
                                   end
                               else 
                                   next_state_var <= water_level_selection;
                           end

                       //duration selecion stage (updating timer value)

                       duration:
                           begin
                               dura_op_reg <= 1'b1;
                               waiting <= 1'b1;
                               if(low)
                                   begin
                                       timer <= 2'd1;
                                   end
                               else if(med)
                                   begin
                                       timer <= 2'd2;
                                   end
                               else if(hig)
                                   begin
                                       timer <= 2'd3;
                                   end
                               else if(next_state_var == wash |(timer != 2'b0 && (!low & !med & !hig)))
                                   begin
                                       next_state_var <= wash;
                                       //count_done <= 1'b0;
                                       waiting <= 1'b0;
                                   end
                               else 
                                   next_state_var <= duration;
                           end
                       wash:
                           begin
                               //spinning <= 1'b1;
                               prog_op_reg <= 1'b0;
                               if(counter_status == 2'd2)
                                   begin
                                       next_state_var <= rinse;
                                   end
                               else
                                   begin
                                       next_state_var <= wash;
                                   end
                           end
                       rinse:
                           begin
                               level_op_reg <= 1'b0;
                               temp_op_reg <= 1'b0;
                               if(counter_status == 2'd2)
                                   begin
                                       next_state_var <= dry;
                                   end
                               else
                                   begin
                                       next_state_var <= rinse;
                                   end
                           end
                       dry:
                           begin
                               if(counter_status == 2'd2)
                                   begin
                                       dura_op_reg <= 1'b0;
                                       next_state_var <= 3'dx;
                                       timer <= 2'b0;
                                       Program <= 2'b0;
                                       temp <= 2'b0;
                                       level <= 2'b0;
                                       //counter_status <= 2'b0;
                                       next_state_var <= 3'b0;
                                       //count_done = 1'b0;
                                       //spinning <= 1'b0;
                                   end
                               else
                                   begin
                                       next_state_var <= dry;
                                   end
                           end
                           //default
                       default: 
                           //this state can be used for pause as well 
                           next_state_var <= next_state_var;

                   endcase
               end
           
       end

   //------------------------------------------------------------counter----------------------------------------------------


       //set appropriate counter values: 
       reg [11:0] counter1 ;
       reg [31:0] counter2 ;

       //always @(posedge clk or posedge reset) begin
       always @(posedge clk) begin
           if(state_var == 3'b0)
              spinning <= 1'b0;
           else 
              spinning <= spinning;
           if(reset)
           begin
               counter1 = 12'b0;
               counter2 <= 32'b0;
               max_count <= 12'b0;
               pouring <= 1'b0;
               heating <= 1'b0;
               spinning <= 1'b0;
               rinse_status <= 1'b0;
               dry_status <= 1'b0;
               counter_status <= 2'b0;
           end
           else
               begin
                   case (state_var)
                       wash:
                           begin
                               if(timer == 2'd1 && counter_status == 2'b0)
                                   begin
                                       counter1 = low_timer; 
                                       if((temp==2'd1)&&(level==2'd1))
                                           counter1 = counter1 + low_pouring;
                                       else if((temp==2'd1)&&(level==2'd2))
                                           counter1 = counter1 + med_pouring;
                                       else if((temp==2'd1)&&(level==2'd3))
                                           counter1 = counter1 + hig_pouring;
                                       else if((temp==2'd2)&&(level==2'd1))
                                           counter1 = counter1 + med_heating + low_pouring;
                                       else if((temp==2'd2)&&(level==2'd2))
                                           counter1 = counter1 + med_heating + med_pouring;
                                       else if((temp==2'd2)&&(level==2'd3))
                                           counter1 = counter1 + med_heating + hig_pouring;
                                       else if((temp==2'd3)&&(level==2'd1))
                                           counter1 = counter1 + hig_heating + low_pouring;
                                       else if((temp==2'd3)&&(level==2'd2))
                                           counter1 = counter1 + hig_heating + med_pouring;
                                       else if((temp==2'd3)&&(level==2'd3))
                                           counter1 = counter1 + hig_heating + hig_pouring;
                                       else
                                           counter1 = counter1;

                                       max_count <= counter1;

                                   end
                               else if(timer == 2'd2 && counter_status == 2'b0)
                                   begin
                                       counter1 = med_timer;
                                       if((temp==2'd1)&&(level==2'd1))
                                           counter1 = counter1 + low_pouring;
                                       else if((temp==2'd1)&&(level==2'd2))
                                           counter1 = counter1 + med_pouring;
                                       else if((temp==2'd1)&&(level==2'd3))
                                           counter1 = counter1 + hig_pouring;
                                       else if((temp==2'd2)&&(level==2'd1))
                                           counter1 = counter1 + med_heating + low_pouring;
                                       else if((temp==2'd2)&&(level==2'd2))
                                           counter1 = counter1 + med_heating + med_pouring;
                                       else if((temp==2'd2)&&(level==2'd3))
                                           counter1 = counter1 + med_heating + hig_pouring;
                                       else if((temp==2'd3)&&(level==2'd1))
                                           counter1 = counter1 + hig_heating + low_pouring;
                                       else if((temp==2'd3)&&(level==2'd2))
                                           counter1 = counter1 + hig_heating + med_pouring;
                                       else if((temp==2'd3)&&(level==2'd3))
                                           counter1 = counter1 + hig_heating + hig_pouring;
                                       else
                                           counter1 = counter1;

                                       max_count <= counter1;

                                   end
                               else if(timer == 2'd3 && counter_status == 2'b0)
                                   begin
                                       counter1 = hig_timer; 
                                       if((temp==2'd1)&&(level==2'd1))
                                           counter1 = counter1 + low_pouring;
                                       else if((temp==2'd1)&&(level==2'd2))
                                           counter1 = counter1 + med_pouring;
                                       else if((temp==2'd1)&&(level==2'd3))
                                           counter1 = counter1 + hig_pouring;
                                       else if((temp==2'd2)&&(level==2'd1))
                                           counter1 = counter1 + med_heating + low_pouring;
                                       else if((temp==2'd2)&&(level==2'd2))
                                           counter1 = counter1 + med_heating + med_pouring;
                                       else if((temp==2'd2)&&(level==2'd3))
                                           counter1 = counter1 + med_heating + hig_pouring;
                                       else if((temp==2'd3)&&(level==2'd1))
                                           counter1 = counter1 + hig_heating + low_pouring;
                                       else if((temp==2'd3)&&(level==2'd2))
                                           counter1 = counter1 + hig_heating + med_pouring;
                                       else if((temp==2'd3)&&(level==2'd3))
                                           counter1 = counter1 + hig_heating + hig_pouring;
                                       else
                                           counter1 = counter1;

                                       max_count <= counter1;

                                   end
                               else if((counter1 != 12'b0) && (counter2 == 32'b0))
                                   begin
                                       counter1 = counter1 - 12'b1;
                                   end
                               else
                                   begin
                                       counter1 = counter1;
                                       //code the conditions for heating and pouring here 

                                       if(temp==2'd1 && level==2'd1)
                                           begin
                                               if((max_count - counter1) <=  low_pouring )
                                                   begin
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                       heating <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd1 && level==2'd2)
                                           begin
                                               if((max_count - counter1) <= med_pouring)
                                                   begin
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                       heating <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd1 && level==2'd3)
                                           begin
                                               if((max_count - counter1) <= hig_pouring )
                                                   begin
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                       heating <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd2 && level==2'd1)
                                           begin
                                               if((max_count - counter1) <= med_heating )
                                                   begin
                                                       heating <= 1'b1;
                                                       spinning <= 1'b0;
                                                       pouring <= 1'b0;
                                                   end
                                               else if((max_count - counter1) <= (med_heating + low_pouring) )
                                                   begin
                                                       heating <= 1'b0;
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd2 && level==2'd2)
                                           begin
                                               if((max_count - counter1) <= med_heating )
                                                   begin
                                                       heating <= 1'b1;
                                                       spinning <= 1'b0;
                                                       pouring <= 1'b0;
                                                   end
                                               else if((max_count - counter1) <= (med_heating + med_pouring))
                                                   begin
                                                       heating <= 1'b0;
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd2 && level==2'd3)
                                           begin
                                               if((max_count - counter1) <= med_heating )
                                                   begin
                                                       heating <= 1'b1;
                                                       spinning <= 1'b0;
                                                       pouring <= 1'b0;
                                                   end
                                               else if((max_count - counter1) <= (med_heating + hig_pouring) )
                                                   begin
                                                       heating <= 1'b0;
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd3 && level==2'd1)
                                           begin
                                               if((max_count - counter1) <= hig_heating )
                                                   begin
                                                       heating <= 1'b1;
                                                       spinning <= 1'b0;
                                                       pouring <= 1'b0;
                                                   end
                                               else if((max_count - counter1) <= (hig_heating + low_pouring) )
                                                   begin
                                                       heating <= 1'b0;
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd3 && level==2'd2)
                                           begin
                                               if((max_count - counter1) <= hig_heating)
                                                   begin
                                                       heating <= 1'b1;
                                                       spinning <= 1'b0;
                                                       pouring <= 1'b0;
                                                   end
                                               else if((max_count - counter1) <= (hig_heating + med_pouring))
                                                   begin
                                                       heating <= 1'b0;
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else if(temp==2'd3 && level==2'd3)
                                           begin
                                               if((max_count - counter1) <= hig_heating)
                                                   begin
                                                       heating <= 1'b1;
                                                       spinning <= 1'b0;
                                                       pouring <= 1'b0;
                                                   end
                                               else if((max_count - counter1) <= (hig_heating + hig_pouring))
                                                   begin
                                                       heating <= 1'b0;
                                                       pouring <= 1'b1;
                                                       spinning <= 1'b0;
                                                   end
                                               else
                                                   begin
                                                       pouring <= 1'b0;
                                                       spinning <= 1'b1;
                                                       heating <= 1'b0;
                                                   end
                                           end
                                       else
                                           begin
                                               pouring <= 1'b0;
                                               heating <= 1'b0;
                                               spinning <= 1'b0;
                                           end
                                   end
                           end
                       rinse:
                           begin
                               if((counter1 == 12'b0) && (!rinse_status))
                                   begin
                                       counter1 = 12'd800;
                                       rinse_status <= 1'b1;
                                   end
                               else if((counter1 != 12'b0) && (counter2 == 32'b0))
                                   counter1 = counter1 - 12'b1;
                               else 
                                   counter1 = counter1;
                           end
                       dry:
                           begin
                               if((counter1 == 12'b0) && (!dry_status))
                                   begin
                                       counter1 = 12'd800;
                                       dry_status <= 1'b1;
                                   end
                               else if((counter1 != 12'b0) && (counter2 == 32'b0))
                                   counter1 = counter1 - 12'b1;
                               else 
                                   counter1 = counter1;
                           end
                       default:
                          begin 
                             counter1 = counter1;
                      		  heating <= 1'b0;
                             pouring <= 1'b0;
                 //     		  waiting <= 1'b0;
                      		  spinning <= 1'b0;
                          end
                   endcase

                   if((counter1 != 12'b0)  && (counter2 != 32'b0))
                       counter2 <= counter2 - 32'b1;
                   else if((counter1 != 12'b0) && (counter2 == 32'b0))
                       counter2 <= 32'h00ffffff;//32'd8; //32'hffffffff;
                   else 
                       counter2 <= 32'h00ffffff;//32'd8; //32'hffffffff;

                   //COUNTER TERMINATING CONDITIONS

                   if((counter1 == 12'b0) && ((state_var == wash) | (state_var == rinse) | (state_var == dry)) && (counter_status == 2'd1))
                       counter_status <= 2'd2;
                   else if((state_var == wash) | (state_var == rinse) | (state_var == dry))
                       counter_status <= 2'b1;
                   else if(prev_state != state_var)
                       counter_status <= 2'b0;
                   else
                       counter_status <= 2'b0;
               end

       end

       //reseting the state variable

      /* always @(state_var) begin

           counter_status <= 2'b0;

       end */
     
       //OUTPUT ASSIGNMENT 
       assign heating_op = heating;
       assign pouring_op = pouring;
       assign waiting_op = waiting;
       assign spinning_op = spinning;
       assign prog_op = prog_op_reg;
       assign level_op = level_op_reg;
       assign temp_op = temp_op_reg;
       assign dura_op = dura_op_reg;

endmodule


\SV
   // Include Tiny Tapeout Lab.
   m4_include_lib(['https:/']['/raw.githubusercontent.com/os-fpga/Virtual-FPGA-Lab/5744600215af09224b7235479be84c30c6e50cb7/tlv_lib/tiny_tapeout_lib.tlv'])


\TLV my_design()
   
   
   
   // ==================
   // |                |
   // | YOUR CODE HERE |
   // |                |
   // ==================
   
   // Note that pipesignals assigned here can be found under /fpga_pins/fpga.
   |fsm
      @1
         $wm_select = *ui_in[0] ;
         $prog_select = *ui_in[7] && !$wm_select;// 0 means lipsi 1 means uart
         $lipsi_select = !$wm_select && !$prog_select;
         
         $imem_rd_addr[3:0] = /top/fpga_pins/fpga|lipsi>>0$pc[3:0];
         $instr[7:0] = $instr_mem;
         $idata_rd_addr[3:0] = /top/fpga_pins/fpga|lipsi>>0$dptr[3:0];
         $data[7:0] = $data_rd;
         $rd_en = /top/fpga_pins/fpga|lipsi>>0$rd_en;
         
         
         $wr_en = /top/fpga_pins/fpga|lipsi>>0$reset ? /top/fpga_pins/fpga|uart>>0$wr_en : /top/fpga_pins/fpga|lipsi>>0$wr_en;
         $idata_wr_addr[3:0] = /top/fpga_pins/fpga|lipsi>>0$reset ? /top/fpga_pins/fpga|uart>>0$idata_wr_addr[3:0] :/top/fpga_pins/fpga|lipsi>>0$dptr[3:0];
         $data_wr[7:0] = /top/fpga_pins/fpga|lipsi>>0$reset ? /top/fpga_pins/fpga|uart>>0$data_wr : /top/fpga_pins/fpga|lipsi>>0$data_wr;
         
         $instr_wr[7:0] = /top/fpga_pins/fpga|uart>>0$instr_wr[7:0];
         $imem_wr_addr[3:0] = /top/fpga_pins/fpga|uart>>0$imem_wr_addr[3:0];
         $instr_wr_en = /top/fpga_pins/fpga|uart>>0$instr_wr_en;
         $digit[3:0] = /top/fpga_pins/fpga|lipsi>>0$reset ? /top/fpga_pins/fpga|uart>>0$digit : /top/fpga_pins/fpga|lipsi>>0$digit;
         *uo_out[7:0] = $wm_select
               ?/top/fpga_pins/fpga|controller>>0$out:
            $digit[3:0] == 4'b0000
               ? 8'b00111111 :
            $digit[3:0] == 4'b0001
               ? 8'b00000110 :
            $digit[3:0] == 4'b0010
               ? 8'b01011011 :
            $digit[3:0] == 4'b0011
               ? 8'b01001111 :
            $digit[3:0] == 4'b0100
               ? 8'b01100110 :
            $digit[3:0] == 4'b0101
               ? 8'b01101101 :
            $digit[3:0] == 4'b0110
               ? 8'b01111101 :
            $digit[3:0] == 4'b0111
               ? 8'b00000111 :
            $digit[3:0] == 4'b1000
               ? 8'b01111111 :
            $digit[3:0] == 4'b1001
               ? 8'b01101111 :
            $digit[3:0] == 4'b1010
               ? 8'b01110111 :
            $digit[3:0] == 4'b1011
               ? 8'b01111100 :
            $digit[3:0] == 4'b1100
               ? 8'b00111001 :
            $digit[3:0] == 4'b1101
               ? 8'b01011110 :
            $digit[3:0] == 4'b1110
               ? 8'b01111001 : 8'b01110001 ;
      m5+imem(@1)
      
   |controller
      @1
         $reset = !/top/fpga_pins/fpga|fsm>>0$wm_select || *reset ;
         $clk = *clk;
         $start = *ui_in[1];
         $low = *ui_in[2];
         $med = *ui_in[3];
         $hig = *ui_in[4];
         
         \SV_plus
            Controller C1(.clk($clk),
                        .reset($reset),
                        .start($start),
                        .low($low),
                        .hig($hig),
                        .med($med),
                        .heating_op($$heat),
                        .spinning_op($$spin),
                        .pouring_op($$pour),
                        .waiting_op($$wait),
                        .prog_op($$prog),
                        .level_op($$level),
                        .temp_op($$temp),
                        .dura_op($$dura));
         $out[7:0] = {$heat,$spin,$pour,$wait,$prog,$level,$temp,$dura};
   
   |uart
      @1
         
         $pc[3:0] = $reset
                     ? 4'b0:
                  $instr_wr_en
                     ?>>1$pc+1:
                     >>1$pc;
         
         $dptr[3:0] = $reset
                     ? 4'b0:
                  $wr_en
                     ?>>1$dptr+1:
                     >>1$dptr;
         
         $reset = !/top/fpga_pins/fpga|fsm>>0$prog_select || *reset ;
         
         $rx_serial = *ui_in[6];   // pmod connector's TxD port
         
         $prog_mem = *ui_in[5];//0 means data 1 means instruction
         
         \SV_plus
            uart_rx #(20000000,115200) uart_rx_1(.clk(*clk),
                                            .reset($reset),
                                            .rx_serial($rx_serial),
                                            .rx_done($$rx_done),
                                            .rx_byte($$rx_byte[7:0])
                                            );
         $first_byte = $reset ? 1'b1 : >>1$first_byte + $rx_done;
         $data[7:0] = (($rx_byte >= 8'h41 && $rx_byte <= 8'h46) || ($rx_byte >= 8'h61 && $rx_byte <= 8'h66))&& $rx_done && >>1$first_byte
                        ? {($rx_byte[3:0] - 4'h7) , 4'b0}:
                     $rx_done && >>1$first_byte
                        ?{$rx_byte[3:0],4'b0}:
                     (($rx_byte >= 8'h41 && $rx_byte <= 8'h46) || ($rx_byte >= 8'h61 && $rx_byte <= 8'h66)) && $rx_done
                        ? {>>1$data[7:4],($rx_byte[3:0] - 4'h7)}:
                     $rx_done
                        ?{>>1$data[7:4],$rx_byte[3:0]}:
                        >>1$data[7:0];
         
         $imem_wr_addr[3:0] = >>1$pc[3:0];
         $instr_wr_en = $rx_done && !>>1$first_byte && !$reset && $prog_mem;
         $instr_wr[7:0] = $data;
         $wr_en = $rx_done && !>>1$first_byte && !$reset && !$prog_mem;
         $idata_wr_addr[3:0] = >>1$dptr[3:0];
         $data_wr[7:0] = $data;
         $digit[3:0] = *ui_in[1] ? $data[7:4]:$data[3:0];
  
   |lipsi
      @1
         
         $reset = *reset || !/top/fpga_pins/fpga|fsm>>0$lipsi_select;
         //---------------------MEMORY - INITIALIZATION---------------
         
         $instr[7:0] = /top/fpga_pins/fpga|fsm>>0$instr;
         $data[7:0] = /top/fpga_pins/fpga|fsm>>0$data;
         
         //-----------------------PC - LOGIC -------------------------
         $pc[7:0] = $reset || >>1$reset
                       ? 8'b0:
                    >>1$exit || >>1$is_ld_ind || >>1$is_st_ind 
                       ? >>1$pc:
                    >>2$is_br || (>>2$is_brz && >>1$z) || (>>2$is_brnz && !>>1$z)
                       ? >>1$instr:
                    >>1$is_brl
                       ? >>1$acc:
                    >>1$is_ret
                       ? >>1$data+1'b1:
                     >>1$pc + 8'b1;
         //---------------------DECODE - LOGIC -----------------------
         $valid = (1'b1^>>1$is_2cyc) && !$reset;
         
         $is_ALU_reg = $instr[7] == 0 && $valid;
         $is_st = $instr[7:4] == 4'b1000 && $valid ;
         $is_brl = $instr[7:4] == 4'b1001 && $valid ;
         $is_ret = {$instr[7:4], $instr[1:0]} == 6'b1101_01 && $valid;
         $is_ld_ind = $instr[7:4] == 4'b1010 && $valid;
         $is_st_ind = $instr[7:4] == 4'b1011 && $valid;
         $is_sh = $instr[7:4] == 4'b1110 && $valid;
         //$is_io = $instr[7:4] == 4'b1111 && $instr[7:0]!=8'b1111_1111 && $valid;
         $exit = $instr[7:0] == 8'b1111_1111 && $valid;
         $is_ALU_imm = $instr[7:4] == 4'b1100 && $valid;
         $is_br = {$instr[7:4], $instr[1:0]} == 6'b1101_00 && $valid;
         $is_brz = {$instr[7:4], $instr[1:0]} == 6'b1101_10 && $valid;
         $is_brnz = {$instr[7:4], $instr[1:0]} == 6'b1101_11 && $valid;
         $is_2cyc = ($is_ALU_imm || $is_br || $is_ld_ind || $is_st_ind || $is_brz || $is_brnz);
         //---------------------ALU - OPERATIONS---------------------
         $func[2:0] = $is_ALU_reg
                         ? $instr[6:4] :
                      >>1$is_ALU_imm
                         ? >>1$instr[2:0] :
                      3'bxxx;
         
         $dptr[7:0] = $reset
                    ? 8'b0:
                 $is_ALU_reg || $is_ld_ind || $is_st || $is_st_ind || $is_brl
                    ? {4'b0,$instr[3:0]}:
                 $is_brl
                    ?{6'b1111_11 ,$instr[1:0]}:
                 $is_ret
                    ?{6'b1111_11 ,$instr[3:2]}:
                 >>1$is_ld_ind  || >>1$is_st_ind 
                    ? >>1$data:
                    >>1$dptr;
         
         $rd_en = $is_ALU_reg || $is_ld_ind || >>1$is_ld_ind || $is_st_ind || $is_ret;
         $wr_en = $is_st || >>1$is_st_ind || $is_brl;
         $op[7:0] = >>1$is_ALU_imm
                       ? $instr :
                    $is_ALU_reg
                       ? $data:
                    8'bxx;
         $is_ALU = >>1$is_ALU_imm || $is_ALU_reg;
         
         /* verilator lint_off WIDTHEXPAND */
         {$c,$acc[7:0]} = $reset
                           ? 9'b0:
                        $is_ALU && $func == 3'b000
                           ? >>1$acc + $op[7:0] :
                        $is_ALU && $func == 3'b000
                           ? >>1$acc + $op[7:0] :
                        $is_ALU && $func == 3'b001
                           ? >>1$acc - $op[7:0] :
                        $is_ALU && $func == 3'b010
                           ? >>1$acc + $op[7:0] + >>1$c :
                        $is_ALU && $func == 3'b011
                           ? >>1$acc - $op[7:0] - >>1$c :
                        $is_ALU && $func == 3'b100
                           ? {>>1$c, >>1$acc & $op[7:0]} :
                        $is_ALU && $func == 3'b101
                           ? {>>1$c, >>1$acc | $op[7:0]}:
                        $is_ALU && $func == 3'b110
                           ? {>>1$c, >>1$acc ^ $op[7:0]} :
                        $is_ALU && $func == 3'b111
                           ? {>>1$c, $op[7:0]}:
                        $is_sh && $instr[1:0] == 2'b00
                           ? {>>1$acc[7:0],>>1$c}:
                        $is_sh && $instr[1:0] == 2'b01
                           ? {>>1$acc[0],>>1$c,>>1$acc[7:1]}:
                        $is_sh && $instr[1:0] == 2'b10
                           ? {>>1$c,>>1$acc[6:0],>>1$acc[7]}:
                        $is_sh && $instr[1:0] == 2'b11
                           ? {>>1$c,>>1$acc[0],>>1$acc[7:1]}:
                        >>1$is_ld_ind
                           ? {>>1$c,$data}:
                         {>>1$c,>>1$acc[7:0]};
         
         /* verilator lint_on WIDTHEXPAND */
         $z = $acc == 8'b0;
         //$data_wr[7:0] = $wr_en? $acc : >>1$data_wr;
         $data_wr[7:0] = !$wr_en ? >>1$data_wr:
                         !$is_brl ? $acc:
                         $pc;
         $digit[3:0] = *ui_in[1]? $acc[7:4] : $acc[3:0];
         
         
         
      //m5+imem(@1)
   
   
   
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
   
   assign passed = top.cyc_cnt > 800;
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
