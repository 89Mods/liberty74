module dice(input i_clk, input RST, input ROLL, output [7:0] LEDS);

(* keep *) PINOUT_8 i_pin8_LEDs(
	.TO_HEADER(LEDS)
);

(* keep *) PINOUT_8 i_pin8_ctrl(
	.TO_HEADER({3'b000, i_clk, 1'b0, RST, ROLL, 1'b0})
);

(* keep *) PULLDOWN_R0603 i_pulldown_rst(
	.Y(RST)
);

(* keep *) PULLDOWN_R0603 i_pulldown_roll(
	.Y(ROLL)
);

	reg[2:0] bcd;
	reg[6:0] segments;
	always @(*) begin
		case(bcd)
			0: segments = 7'b0111111;
			1: segments = 7'b0000110;
			2: segments = 7'b1011011;
			3: segments = 7'b1001111;
			4: segments = 7'b1100110;
			5: segments = 7'b1101101;
			6: segments = 7'b1111100;
			7: segments = 7'b0000111;
		endcase
	end
	assign LEDS[6:0] = segments[6:0];
	reg rolling;
	reg [15:0] lfsr;
	reg [7:0] clkdiv;
	reg [7:0] counter;
	reg dp;
	assign LEDS[7] = dp;
	
	always @(posedge i_clk) begin
		if(RST) begin
			rolling = 0;
			lfsr[15:8] = 0;
			lfsr[7:0] = 8'b11011010;
			clkdiv = 8'b10100000;
			bcd = 1;
			dp = 1;
		end else begin
			lfsr[9:0] = lfsr[10:1];
			lfsr[10] = lfsr[11] ^ lfsr[0];
			lfsr[11] = lfsr[12];
			lfsr[12] = lfsr[13] ^ lfsr[0];
			lfsr[13] = lfsr[14] ^ lfsr[0];
			lfsr[14] = lfsr[15];
			lfsr[15] = lfsr[0];
			
			if(ROLL) begin
				clkdiv = 2;
				counter = 0;
				dp = 0;
			end
			if(clkdiv != 8'b10100000) begin
				counter = counter + 1;
				if(counter == clkdiv) begin
					counter = 0;
					clkdiv = clkdiv + 1;
					if(lfsr[2:0] > 5) begin
						bcd[2:0] = lfsr[2:0] - 4;
					end else begin
						bcd[2:0] = lfsr[2:0] + 1;
					end
				end
			end else begin
				dp = 1;
			end
		end
	end
endmodule
