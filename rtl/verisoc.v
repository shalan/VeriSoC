`timescale 1 ns / 1 ps


module verisoc #(
	parameter VERBOSE = 0
) (
	input           wb_clk,
	input           wb_rst,
	output          trap,
	
    // WB Ports
    output [31:0]   wb_m2s_adr,
	output [31:0]   wb_m2s_dat,
	output [3:0]    wb_m2s_sel,
	output          wb_m2s_we,
	output          wb_m2s_cyc,
	output          wb_m2s_stb,
	input [31:0]    wb_s2m_dat,
	input           wb_s2m_ack,

    // AHB Lite Ports
    input           hresetn,		 
    input           hclk,		 
    input [31:0]    hrdata,		

    input [1:0]     hresp,
    input           hready,

    output [31:0]   haddr,	
    output          hwrite,	
    output [2:0]    hsize,	
    output [2:0]    hburst,	
    output [31:0]   hwdata,	
    output [1:0]    htrans	
);

//	wire tests_passed;
	reg [31:0] irq = 0;
//	wire mem_instr;
/*
	reg [15:0] count_cycle = 0;
	always @(posedge wb_clk) count_cycle <= !wb_rst ? count_cycle + 1 : 0;

	always @* begin
		irq = 0;
		irq[4] = &count_cycle[12:0];
		irq[5] = &count_cycle[15:0];
	end


	wire [31:0] wb_m2s_adr;
	wire [31:0] wb_m2s_dat;
	wire [3:0]  wb_m2s_sel;
	wire        wb_m2s_we;
	wire        wb_m2s_cyc;
	wire        wb_m2s_stb;
	*/
    wire [31:0] wb_dat;
	wire        wb_ack;



    wire [31:0] wbs_dat_ram_o, wbs_dat_wb_o, wbs_dat_ahb_o;;
    wire        wbs_stb_ram_i, wbs_stb_wb_i, wbs_stb_ahb_i;
    wire        wbs_ack_ram_o, wbs_ack_wb_o, wbs_ack_ahb_o;

    assign      wbs_stb_ram_i   =   wb_m2s_stb & ~wb_m2s_adr[14];
    assign      wbs_stb_wb_i    =   wb_m2s_stb & wb_m2s_adr[14] & ~wb_m2s_adr[13]; 
    assign      wbs_stb_ahb_i   =   wb_m2s_stb & wb_m2s_adr[14] & wb_m2s_adr[13]; 



    assign      wb_ack   =   wbs_stb_ram_i ? wbs_ack_ram_o :
                                wbs_stb_wb_i ? wb_s2m_ack :
                                wbs_stb_ahb_i ? wbs_ack_ahb_o :
                                1'b1;

    assign      wb_dat   =   wbs_stb_ram_i ? wbs_dat_ram_o :
                                wbs_stb_wb_i ? wb_s2m_dat :
                                wbs_stb_ahb_i ? wbs_dat_ahb_o :
                                32'hDEADBEEF;


    ahbmas_wbslv #( 32 , 32  ) 
    WBS2AHBM ( 
        .htrans (htrans) ,
        .hwrite (hwrite) ,
        .hrdata (hrdata) ,
        .hready (hready ) ,
        .hburst (hburst ) ,
        .hwdata (hwdata ) ,
        .hresp  (hresp ) ,
        .haddr  (haddr ) ,
        .hsize  (hsize ) ,
        
        .stb_i  (wbs_stb_ahb_i) ,
        .data_i (wb_m2s_dat) ,
        .sel_i  (wb_m2s_sel) ,
        .ack_o  (wbs_ack_ahb_o) ,
        .data_o (wbs_dat_ahb_o ) ,
        .addr_i (wb_m2s_adr ) ,
        .we_i   (wb_m2s_we ) ,
        .cyc_i  (wb_m2s_cyc ) ,
        
        .clk_i  (wb_clk),
        .rst_i  (wb_rst)
        //,
        //.hclk (hclk),
        //.hresetn(hresetn)
    ); 

	wb_mem #(
		.NUM_WORDS (4*1024),
		.VERBOSE (VERBOSE)
	) ram ( // Wishbone interface
		.wb_clk_i(wb_clk),
		.wb_rst_i(wb_rst),

		.wb_adr_i(wb_m2s_adr),
		.wb_dat_i(wb_m2s_dat),
		.wb_stb_i(wbs_stb_ram_i),
		.wb_cyc_i(wb_m2s_cyc),
		.wb_dat_o(wbs_dat_ram_o),
		.wb_ack_o(wbs_ack_ram_o),
		.wb_sel_i(wb_m2s_sel),
		.wb_we_i(wb_m2s_we)
/*
		.mem_instr(mem_instr),
		.tests_passed(tests_passed)
*/
	);

	picorv32_wb #(
		.ENABLE_REGS_DUALPORT(1),
		.COMPRESSED_ISA(1),
		.ENABLE_MUL(1),
		.ENABLE_DIV(1),
		.ENABLE_IRQ(1),
		.ENABLE_TRACE(0)
    ) cpu (
		.trap (trap),
		.irq (irq),
		//.trace_valid (trace_valid),
		//.trace_data (trace_data),
		//.mem_instr(mem_instr),

		.wb_clk_i(wb_clk),
		.wb_rst_i(wb_rst),

		.wbm_adr_o(wb_m2s_adr),
		.wbm_dat_i(wb_dat),
		.wbm_stb_o(wb_m2s_stb),
		.wbm_ack_i(wb_ack),
		.wbm_cyc_o(wb_m2s_cyc),
		.wbm_dat_o(wb_m2s_dat),
		.wbm_we_o(wb_m2s_we),
		.wbm_sel_o(wb_m2s_sel)
	);
/*
	reg [1023:0] firmware_file;
	initial begin
		if (!$value$plusargs("firmware=%s", firmware_file))
			firmware_file = "firmware/firmware.hex";
		$readmemh(firmware_file, ram.mem);
	end

	integer cycle_counter;
	always @(posedge wb_clk) begin
		cycle_counter <= !wb_rst ? cycle_counter + 1 : 0;
		if (!wb_rst && trap) begin
`ifndef VERILATOR
			repeat (10) @(posedge wb_clk);
`endif
			$display("TRAP after %1d clock cycles", cycle_counter);
			if (tests_passed) begin
				$display("ALL TESTS PASSED.");
				$finish;
			end else begin
				$display("ERROR!");
				if ($test$plusargs("noerror"))
					$finish;
				$stop;
			end
		end
	end
*/
endmodule

module wb_mem #(
	parameter NUM_WORDS = 4*1024,
    parameter DMEM_ADDR = 8*1024,
	parameter HEX_FILE  = "",
	parameter VERBOSE   = 0
) (
	input wb_clk_i,
	input wb_rst_i,

	input [31:0]        wb_adr_i,
	input [31:0]        wb_dat_i,
	input [3:0]         wb_sel_i,
	input               wb_we_i,
	input               wb_cyc_i,
	input               wb_stb_i,
	output reg          wb_ack_o,
	output reg [31:0]   wb_dat_o
);

	reg [31:0]  adr_r;
	wire        valid = wb_cyc_i & wb_stb_i;

	always @(posedge wb_clk_i or posedge wb_rst_i) begin
		if (wb_rst_i) begin
			adr_r <= {32{1'b0}};
			wb_ack_o <= 1'b0;
        end else begin
            adr_r <= wb_adr_i;
		    wb_ack_o <= valid & !wb_ack_o;
        end
	end

	wire        ram_we  = wb_we_i & valid & wb_ack_o;
	wire [31:0] waddr   = adr_r[31:2];
	wire [31:0] raddr   = wb_adr_i[31:2];
	wire [3:0]  we      = {4{ram_we}} & wb_sel_i;

	reg [31:0] mem [NUM_WORDS-1:0];

	always @(posedge wb_clk_i) begin
		if (ram_we & VERBOSE)
				$display("WR: ADDR=%08x DATA=%08x STRB=%04b", adr_r, wb_dat_i, we);
        if (valid & wb_ack_o & !ram_we)
			if (VERBOSE)
				$display("RD: ADDR=%08x DATA=%08x%s", adr_r, mem[raddr2], mem_instr ? " INSN" : "");
    end

    // Memory Write
    always @(posedge wb_clk_i) begin
        if (we[0]) mem[waddr][7:0] <= wb_dat_i[7:0];
        if (we[1]) mem[waddr][15:8] <= wb_dat_i[15:8];
        if (we[2]) mem[waddr][23:16] <= wb_dat_i[23:16];
        if (we[3]) mem[waddr][31:24] <= wb_dat_i[31:24];

		wb_dat_o <= mem[raddr];    
	end

	initial begin
		if (HEX_FILE != "")
			$readmemh(memfile, mem);
	end
endmodule