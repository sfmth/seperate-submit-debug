 
`default_nettype none
// `timescale 1ns/1ns
`define MPRJ_IO_PADS 38   

// `include "/home/farhad/Projects/indian_guy_last_minute_tapeout/projtes/wrapped_ibnalhaytham.v"
// `include "/home/farhad/Projects/indian_guy_last_minute_tapeout/projtes/wrapped_tiny_cordic.v"


module projtes (
    `ifdef USE_POWER_PINS

    inout vccd1,	// User area 1 1.8V supply

    inout vssd1,	// User area 1 digital ground

    `endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,


    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    // inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2

    // User maskable interrupt signals
    // output [2:0] user_irq

    );

      // split remaining 96 logic analizer wires into 3 chunks                                                                                              
    wire [31: 0] la1_data_in, la1_data_out, la1_oenb;                                                                                                     
    assign la1_data_in = la_data_in[63:32];                                                                                                               
    assign la_data_out[63:32] = la1_data_out;                                                                                                             
    assign la1_oenb = la_oenb[63:32];                                                                                                                     
                                                                                                                                                          
    wire [31: 0] la2_data_in, la2_data_out, la2_oenb;                                                                                                     
    assign la2_data_in = la_data_in[95:64];                                                                                                               
    assign la_data_out[95:64] = la2_data_out;                                                                                                             
    assign la2_oenb = la_oenb[95:64];                                                                                                                     
                                                                                                                                                          
    wire [31: 0] la3_data_in, la3_data_out, la3_oenb;                                                                                                     
    assign la3_data_in = la_data_in[127:96];                                                                                                              
    assign la_data_out[127:96] = la3_data_out;                                                                                                            
    assign la3_oenb = la_oenb[127:96];


// generate active wires
    wire [31: 0] active;
    assign active = la_data_in[31:0];


   wrapped_ibnalhaytham wrapped_ibnalhaytham_1(                                                                                                          
        `ifdef USE_POWER_PINS                                                                                                                             
            .vccd1 (vccd1),                                                                                                                                   
            .vssd1 (vssd1),                                                                                                                                   
        `endif                                                                                                                                            
        .wb_clk_i (wb_clk_i),                                                                                                                             
        .active (active[1]),                                                                                                                              
        .la1_data_in (la1_data_in[31:0]),                                                                                                                 
        .la1_data_out (la1_data_out[31:0]),                                                                                                               
        .la1_oenb (la1_oenb[31:0]),                                                                                                                       
        .io_in (io_in[37:0]),                                                                                                                             
        .io_out (io_out[37:0]),                                                                                                                           
        .io_oeb (io_oeb[37:0]),                                                                                                                           
        .user_clock2 (user_clock2)                                                                                                                        
    ); 

    wrapped_tiny_cordic wrapped_tiny_cordic1(
        `ifdef USE_POWER_PINS                                                                                                                             
            .vccd1 (vccd1),                                                                                                                                   
            .vssd1 (vssd1),                                                                                                                                   
        `endif                                                                                                                                            
        .wb_clk_i (wb_clk_i),                                                                                                                             
        .active (active[2]),                                                                                                                              
        .la1_data_in (la1_data_in[31:0]),                                                                                                                 
        .la1_data_out (la1_data_out[31:0]),                                                                                                               
        .la1_oenb (la1_oenb[31:0]),                                                                                                                       
        .io_in (io_in[37:0]),                                                                                                                             
        .io_out (io_out[37:0]),                                                                                                                           
        .io_oeb (io_oeb[37:0]),                                                                                                                           
        .user_clock2 (user_clock2)
    );
    
    // `ifdef FORMAL
    //     initial assume(reset);
    //     // initial assume(reg_file[0] == 32'b0);
    //     always @(posedge clk) begin
    //         cover (reg_file[0] != 0);
    //     end
    // `endif

    // `ifdef COCOTB_SIM
    // initial begin
    // $dumpfile ("data_memory.vcd");
    // $dumpvars (0, data_memory);
    // #1;
    // end
    // `endif
endmodule
 
 
 
