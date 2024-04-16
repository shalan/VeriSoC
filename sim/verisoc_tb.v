module verisoc_tb;

    reg             wb_clk = 0;
    reg             wb_rst;
    wire            trap;

    // WB Ports
    wire [31:0]     wb_m2s_adr;
    wire [31:0]     wb_m2s_dat;
    wire [3:0]      wb_m2s_sel;
    wire            wb_m2s_we;
    wire            wb_m2s_cyc;
    wire            wb_m2s_stb;
    wire [31:0]     wb_s2m_dat;
    wire            wb_s2m_ack;

    // AHB Lite Ports
    wire           hresetn;		 
    wire           hclk;		 
    wire [31:0]    hrdata;		

    wire [1:0]     hresp;
    wire           hready;

    wire  [31:0]   haddr;	
    wire           hwrite;	
    wire  [2:0]    hsize;	
    wire  [2:0]    hburst;	
    wire  [31:0]   hwdata;	
    wire  [1:0]    htrans;	

    verisoc DUV (
	    .wb_clk(wb_clk),
	    .wb_rst(wb_rst),
	    .trap(trap),
    
        .wb_m2s_adr(wb_m2s_adr),
	    .wb_m2s_dat(wb_m2s_dat),
	    .wb_m2s_sel(wb_m2s_sel),
	    .wb_m2s_we(wb_m2s_we),
	    .wb_m2s_cyc(wb_m2s_cyc),
	    .wb_m2s_stb(wb_m2s_stb),
	    .wb_s2m_dat(wb_s2m_dat),
	    .wb_s2m_ack(wb_s2m_ack),

        .hresetn(hresetn),
        .hclk(hclk),
        .hrdata(hrdata),

        .hresp(hresp),
        .hready(hready),

        .haddr(haddr),
        .hwrite(hwrite),
        .hsize(hsize),
        .hburst(hburst),
        .hwdata(hwdata),
        .htrans(htrans)
    );

    // clock
    always #10 wb_clk = !wb_clk;

    // Reset
    initial begin
        wb_rst = 1'bx;
        #100;
        wb_rst = 1'b1;
        #5_000;
        @(posedge wb_clk);
        wb_rst = 1'b0;
    end

    // Dump the VCD data
    initial begin
        $dumpfile("verisoc_tb.vcd");
        $dumpvars;
    end

    // End the simulation
    initial #1_000_000 $finish;
endmodule