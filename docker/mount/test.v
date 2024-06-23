module test(
    input i_clk,
    output LED
    );

    Led_Res_0603 i_result_led_0 (
        .I (LED)
    );

integer counter;
reg state;
always @ (posedge i_clk) begin
    counter <= counter + 1;
     if(counter >= 8000000 )begin
        counter <=0;
        state <= !state;
     end 
end 
assign LED = state;        
endmodule
