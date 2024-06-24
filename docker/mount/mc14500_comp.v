`default_nettype none

module mc14500_comp(
	input i_clk,
	input rst_n,
	input [7:0] io_in,
	input SDI,
	output [22:0] io_out,
	output [7:0] sram_addr,
	output [7:0] sram_in,
	input [7:0] sram_out,
	output sram_gwe,
	input custom_setting
);

PININ_8 pin8_io(
	.FROM_HEADER(io_in)
);

PINOUT_8 pin8_sramaddr(
	.TO_HEADER(sram_addr)
);

PININ_8 pin8_sramin(
	.FROM_HEADER(sram_in)
);

PINOUT_8 pin8_sramout(
	.TO_HEADER(sram_out)
);

PINOUT_8 pin8_ctrl(
	.TO_HEADER({3'b000, i_clk, rst_n, SDI, sram_gwe, custom_setting})
);

wire X1;
wire RR;
wire WRITE;
wire DATA_OUT;
wire JMP;
wire RTN;
wire FLAG_O;
wire FLAG_F;

reg out_1;
reg out_2;
reg clk_div;
reg [16:0] dest;
reg [16:0] PC;
assign io_out[16:0] = PC;
assign io_out[17] = FLAG_O;
assign io_out[18] = RR;
assign io_out[19] = SCLK;
assign io_out[20] = SDO;
assign io_out[21] = out_1;
assign io_out[22] = out_2;
wire [3:0] addr = io_in[7:4];

reg [7:0] mar;
wire [7:0] mar_adj = {mar[0], mar[1], mar[2], mar[3], mar[4], mar[5], mar[6], mar[7]};
reg [7:0] dob;
reg [7:0] dia;
reg [7:0] dib;
reg [1:0] rst_latency;
reg scratch [7:0];
assign sram_addr = mar_adj;
assign sram_in = dob;
assign sram_gwe = !clk_div && FLAG_F;
wire SCLK = scratch[6];
wire SDO = scratch[7];

`ifdef SIM
wire [7:0] scratch_mem = {scratch[7], scratch[6], scratch[5], scratch[4], scratch[3], scratch[2], scratch[1], scratch[0]};

wire [7:0] dob_adj = {dob[0], dob[1], dob[2], dob[3], dob[4], dob[5], dob[6], dob[7]};
`endif

wire DATA_IN = addr == 0 ? 1'b1 : (addr[3] ? scratch[addr[2:0]] : (
	(addr == 3 && dia[7]) || (addr == 4 && dib[7]) || (addr == 7 && SDI)
));

always @(posedge i_clk) begin
	if(!rst_n) begin
		clk_div <= 1'b0;
		dest <= 17'h00000;
		PC <= 17'h00000;
		out_1 <= 1'b1;
		out_2 <= 1'b1;
		mar <= 8'h00;
		dob <= 8'h00;
		dia <= 8'h00;
		dib <= 8'h00;
		scratch[0] <= 1'b0;
		scratch[1] <= 1'b0;
		scratch[2] <= 1'b0;
		scratch[3] <= 1'b0;
		scratch[4] <= 1'b0;
		scratch[5] <= 1'b0;
		scratch[6] <= 1'b0;
		scratch[7] <= 1'b0;
		rst_latency <= 2'b11;
	end else begin
		if(custom_setting) begin
			dest[16] <= 1'b0;
			PC[16] <= 1'b0;
		end
		clk_div <= !clk_div;
		if(rst_latency) rst_latency <= rst_latency - 1;
		if(clk_div && !rst_latency) begin
			if(JMP) PC <= dest;
			else PC <= PC + 1;
			if(RTN) mar <= 8'h00;
			else if(WRITE && addr == 1) mar <= {mar[6:0], DATA_OUT};
			if(WRITE && addr == 0) dest <= {dest[15:0], DATA_OUT};
			if(WRITE && addr == 2) dob <= {dob[6:0], DATA_OUT};
			if(FLAG_O && addr == 1) dia <= sram_out;
			if(FLAG_O && addr == 2) dib <= sram_out;
			if(WRITE && addr[3]) scratch[addr[2:0]] <= DATA_OUT;
			if(WRITE && addr == 5) out_1 <= DATA_OUT;
			if(WRITE && addr == 6) out_2 <= DATA_OUT;
			if(addr == 3) dia <= {dia[6:0], 1'b0};
			if(addr == 4) dib <= {dib[6:0], 1'b0};
		end
	end
end

mc14500 mc14500(
	.X2(clk_div),
	.RST(~rst_n || rst_latency),
	.I(io_in[3:0]),
	.X1(X1),
	.DATA_IN(DATA_IN),
	.DATA_OUT(DATA_OUT),
	.WRITE(WRITE),
	.RR(RR),
	.JMP(JMP),
	.RTN(RTN),
	.FLAG_O(FLAG_O),
	.FLAG_F(FLAG_F)
);

endmodule


`default_nettype none

module mc14500(
	input X2,
	input RST,
	input [3:0] I,

	output X1,
	input DATA_IN,
	output DATA_OUT,
	output WRITE,
	output RR,
	output JMP,
	output RTN,
	output FLAG_O,
	output FLAG_F
);

reg RR_l;
reg IEN_l;
reg OEN_l;

reg [3:0] instr_l;
wire [3:0] instr = (!X2 ? I : instr_l) & {4{~skip}};
reg skip;
reg data_out;

wire g_2_1 = ~( instr[3] |  instr[2]);
wire g_2_2 = ~( instr[3] | ~instr[2]);
wire g_2_3 = ~(~instr[3] |  instr[2]);
wire g_2_4 = ~(~instr[3] | ~instr[2]);

wire g_1_1 = ~( instr[1] |  instr[0]);
wire g_1_2 = ~( instr[1] | ~instr[0]);
wire g_1_3 = ~(~instr[1] |  instr[0]);
wire g_1_4 = ~(~instr[1] | ~instr[0]);

wire NOPO_i = ~(g_2_1 & g_1_1);
wire ANDC_i = ~(g_2_2 & g_1_1);
wire XNOR_i = ~(g_2_2 & g_1_4);
wire IEN_i  = ~(g_2_3 & g_1_3);
wire OEN_i  = ~(g_2_3 & g_1_4);
wire JMP_i  = ~(g_2_4 & g_1_1);
wire RTN_i  = ~(g_2_4 & g_1_2);
wire SKZ_i  = ~(g_2_4 & g_1_3);
wire NOPF_i = ~(g_2_4 & g_1_4);

wire LDC_ORC   = g_1_3;
wire OR_ORC    = ~(g_1_1 | g_1_4) & g_2_2;
wire LD_OR     = g_1_2;
wire AND_XNOR  = g_1_4;
wire update_rr = ~(g_2_3 | g_2_4) & NOPO_i;

//Data bus
wire data = DATA_IN & IEN_l;

//Logic Unit, directly translated from https://static.righto.com/images/mc14500b/alu-diagram.jpg
wire LU_out = ((((~ANDC_i & RR_l) | (~XNOR_i & ~RR_l) | LDC_ORC) & ~data) |
				(OR_ORC & RR_l) |
				((LD_OR ^ (AND_XNOR & RR_l)) & data)
				);
				
//Output signals
assign FLAG_O = ~(NOPO_i | skip);
assign FLAG_F = ~NOPF_i;
assign JMP    = ~JMP_i;
assign RTN    = ~RTN_i;
assign RR     = RR_l;
wire   WE     = ~(~g_2_3 | ~(g_1_1 | g_1_2));
assign WRITE  = WE & OEN_l;
assign X1     = X2;

assign DATA_OUT = data_out;

always @(posedge X2) begin
	//The one thing I’m 100% unsure about is how reset is done in MC14500, but the easiest way I’ve found is to force skip permanently during reset, and AND the RR latch data with reset.
	RR_l  <= (update_rr ? LU_out : RR_l) & ~RST;
	IEN_l <= IEN_i ? IEN_l : DATA_IN;
	OEN_l <= OEN_i ? OEN_l : DATA_IN;
	data_out <= (g_1_1 ? RR_l : ~RR_l) & OEN_l;
	instr_l <= I;
end

always @(negedge X2) begin
	skip  <= ~(SKZ_i | RR_l) | RST;
end

endmodule
