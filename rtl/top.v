`timescale 1 ns / 1 ps
`default_nettype none

module top(
	// bus interface
	input clk_100m,
	input [11:0] addr,
	input fclk, // Clock for serial communication, either 7 or 8 MHz (7 MHz on Apple II)
	input q3, // 2 MHz non-symmetric timing signal
	inout [7:0] data,
	input rw, // 1 means read, 0 means write
	input _iostrobe, // goes low during read or write to any address $C800-$CFFF
	input _iosel, // goes low during read or write to $CX00-$CXFF, where X is slot number
	input _devsel, // goes low during read or write to $C0(X+8)0-$C0(X+8)F, where X is slot number. IWM: Falling edge latches A3-A0. Rising edge of (Q3 or _devsel) qualifies write register data
	input _reset,
	// disk interface
	output wrdata,
	output [3:0] phase,
	output _wrreq,
	output _enbl1,
	inout _enbl2, // output that may be hardwired to ground when connected to a drive 
	inout select, // output that may be hardwired to ground when connected to a drive 
	inout _en35,  // output that may be hardwired to ground when connected to a drive
	input sense,
	input rddata,
	// SPI
	/*output spi_clk,
	output spi_mosi,
	input spi_miso,
	output spi_cs,*/
	// level-shifting buffers
	output _en245, // bidirectional connection of data bus to FPGA
	//output [7:0] dbgDataOut
	// debugging: LEDs or switches or misc stuff
//	output [7:0] debugInfo
	input rx,
	output tx,
	output devsel_n,
	output iostrobe_n,
	output iosel_n,
	output bufen,
	input btn
    );

assign iostrobe_n = latch;
assign iosel_n = latch;
assign devsel_n = latch;
assign bufen = latch;
	wire isOutputting;
	wire romExpansionActive; // 1 if the Yellowstone card's ROM is the currently selected slot ROM
	//assign debugInfo = { romExpansionActive, rw, q3, isOutputting/*_devsel*/, _iosel, _iostrobe, _reset, spi_miso }; //{ romActive, 6'b000000, spi_miso };

	
		wire _romoe;
	
	reg [1:0] en245Delay;
	always @(posedge fclk) begin
		en245Delay <= { en245Delay[0], (~_devsel || ~_romoe) };
	end

	assign _en245 = ~(~q3 &&(en245Delay == 2'b11) && (~_devsel || ~_romoe)); // IWM selected or ROM outputting
	//assign bufen = _en245;


	// IMPORTANT! TO-DO
	// for select, _enbl2, _en35, these outputs may be driven externally to ground!
	// never drive these actively high. configure as inout, enable the internal pull-up, and set output value to 0 or hi-Z
	// may need to pause a few microseconds after setting these to hi-Z to let the pull-up work. It's around 50Kohm equivalent.
	// RC time constant assuming 10 pF trace capacitance is 0.5 microseconds
	
	wire _enbl2_from_iwm;
	assign _enbl2 = 1'bZ;
	assign select = 1'bZ;
	assign _en35 = 1'bZ;

	addrDecoder myAddrDecoder(
		.addr(addr),
		.fclk(fclk),
		._iostrobe(_iostrobe),
		._iosel(_iosel),
		._reset(_reset),
		._romoe(_romoe),
		.romExpansionActive(romExpansionActive)
	);
	
	wire [7:0] iwmDataOut;
	wire [7:0] buffer2;

	wire q6, q7, motor;

	iwm myIwm(
		.addr(addr[3:0]),
		._devsel(_devsel),
		.fclk(fclk),
		.q3(q3),
		._reset(_reset),
		.dataIn(data_in),
		.dataOut(iwmDataOut),
		.wrdata(wrdata),
		.phase(phase),
		._wrreq(_wrreq),
		._enbl1(_enbl1),
		._enbl2(_enbl2_from_iwm),
		.sense(sense),
		.rddata(rddata),
		.q6w(q6),
		.q7w(q7),
		.motor(motor),
		.buffer2(buffer2),
		.q3orDev(latch)
	);


	wire latch;
	 
	wire [7:0] romOutput;
	
	codeROM myROM(
		.clk(fclk), // use internal clock? 
		.Address(addr[11:0]),
		.Reset(0), 
		.Q(romOutput)
	);
	
	reg [1:0] lastDataEnable;
	
	always @(posedge fclk) begin
		lastDataEnable <= { lastDataEnable[0], (rw == 1 && _romoe == 0) || (rw == 1 && _devsel == 0 && addr[0] == 0) };
	end
	
	// provide data from the card's ROM, or the IWM?
	// IWM registers are read during any operation in which A0 is 0
	assign isOutputting = (rw && ~_romoe) /*|| (rw == 1 && _devsel == 0 && addr[0] == 0)*/;
	wire romOut = (lastDataEnable == 2'b11 && rw == 1 && _romoe == 0) || (/*lastDataEnable == 2'b11 &&*/ rw == 1 && _devsel == 0 && addr[0] == 0);
	wire [7:0] dataOut = (lastDataEnable == 2'b11 && rw == 1 && _romoe == 0) ? romOutput :
				  (lastDataEnable == 2'b11 && rw == 1 && _devsel == 0 && addr[0] == 0) ? iwmDataOut : 
				  8'b0;

	wire [7:0] data_in;
	SB_IO #(
		.PIN_TYPE(6'b 1010_01)
	) sram_data_pins [7:0] (
		.PACKAGE_PIN(data),
		.OUTPUT_ENABLE(romOut),
		.D_OUT_0(dataOut),
		.D_IN_0(data_in)
	);

	wire [31:0] events;


	reg [7:0] data_reg;

	always @(posedge clk_100m) begin
		data_reg <= data_in;
	end

	assign events[7:0] = iwmDataOut;
	assign events[11:8] = phase;
	assign events[12] = _reset;
	assign events[13] = q6;
	assign events[14] = q7;
	assign events[15] = motor;
	assign events[16] = fclk;
	assign events[20:17] = addr[3:0];
	assign events[21] = _iosel;
	assign events[22] = _en245;
	assign events[23] = latch;

	sump2_top sump_i(
		.clk_100m(clk_100m),
		.ftdi_wi(rx),
		.ftdi_ro(tx),
		.events_din(events)
	);

endmodule
