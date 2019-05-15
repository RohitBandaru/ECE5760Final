/*****************************************************************************
Rohit Bandaru (rb696), Aasta Gandhi (apg67)
ECE 5760, Spring 2019
3D Marker Tracking, Top Level Module
Code adapted from Terasic Ubuntu Linux Example (details on project page)
	and Bruce Land's VGA Display using Bus Master example
*****************************************************************************/ 

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire			[15: 0]	hex3_hex0;
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;

HexDigit Digit0(HEX0, y_cood[3:0]);
HexDigit Digit1(HEX1, y_cood[7:4]);
HexDigit Digit2(HEX2, y_cood[9:8]);
HexDigit Digit3(HEX3, hex3_hex0[15:12]);

//=======================================================
// VGA Onchip Buffer
//=======================================================

reg [31:0] vga_sram_addr; 
wire [31:0] vga_sram_readdata ;
reg [7:0] vga_sram_writedata ; //[31:0] 
reg vga_sram_write ;
wire vga_sram_clken = 1'b1;
wire vga_sram_chipselect = 1'b1;

//=======================================================
// Bus controller for AVALON bus-master
//=======================================================
reg [31:0] sdram_address ; // Avalon address
wire [31:0] video_base_address = 32'h800_0000 ;  // Avalon address
wire [3:0] sdram_byte_enable  ; // four bit byte read/write mask
reg sdram_read  ;       // high when requesting data
reg sdram_write ;      //  high when writing data
reg [15:0] sdram_write_data ; //  data to send to Avalog bus
reg sdram_ack  ;       //  Avalon bus raises this when done
wire [15:0] sdram_read_data ; // data from Avalon bus
reg [30:0] timer ;

wire state_clock ;

reg [9:0] x_cood, y_cood ;
assign bus_addr = video_base_address + {22'b0,x_cood} + ({22'b0,y_cood}<<10) ;

wire [31:0] vga_out_base_address = 32'h800_0000;

//=======================================================
// FEEDBACK CONTROL 
//=======================================================

//start signal
wire start_signal; 
assign start_signal = SW[4]; //starts system

//set up coordinates coming in from HPS
wire [23:0] marker1_coords; 
wire [23:0] marker2_coords; 
wire [23:0] marker3_coords; 

//parse coordinates
wire [7:0] x1;
assign x1 = marker1_coords[23:16]; 
wire [7:0] y1;
assign y1 = marker1_coords[15:8];
wire [7:0] z1;
assign z1 = marker1_coords[7:0];

wire [7:0] x2;
assign x2 = marker2_coords[23:16]; 
wire [7:0] y2;
assign y2 = marker2_coords[15:8];
wire [7:0] z2;
assign z2 = marker2_coords[7:0];

wire [7:0] x3;
assign x3 = marker3_coords[23:16]; 
wire [7:0] y3;
assign y3 = marker3_coords[15:8];
wire [7:0] z3;
assign z3 = marker3_coords[7:0];

//initialize starting coordinate points 
//for baseline calculations

reg [7:0] z1_start;
reg [7:0] x_start;
reg [7:0] y_start;

reg [7:0] z2_start;
reg [7:0] x2_start;
reg [7:0] y2_start;

reg [7:0] z3_start;
reg [7:0] x3_start;
reg [7:0] y3_start;

reg signed [7:0] z_diff;
wire signed [7:0] thres_pio;

//feedback setup

reg [7:0] feedback_pio; 
wire [1:0] curr_marker; 
reg wrong = 0;
assign LEDR[9] = wrong;
//assign z_diff = y1 - y_start; //set up change in y direction


//callibration logic
reg [6:0] calib_counter = 7'd0; 
reg [2:0] feedback_state = 3'd0;

always @(posedge CLOCK_50) begin

 if(start_signal) begin
	if(~KEY[0])begin //reset 
		feedback_state <= 3'd0; 
		feedback_pio <= 8'd3; 
	end 
	else begin
		if(feedback_state == 3'd0) begin
			if(calib_counter == 7'd0) begin
				x_start <= x1;
				y_start <= y1;
				z1_start <= z1;
				
				z2_start <= z2; 
				z3_start <= z3;
				x2_start <= x2;
				y2_start <= y2;
				
				x3_start <= x3;
				y3_start <= y3;
				
				calib_counter = calib_counter + 7'd1; 
				feedback_pio <= 8'd3;
			end 
			else if(calib_counter < 7'd128) begin
				x_start <= x_start + x1; 
				y_start <= y_start + y1;
				z1_start <= z1_start + z1;
				
				z2_start <= z2_start + z2; 
				z3_start <= z3_start + z3; 
				
				x2_start <= x2_start + x2;
				y2_start <= y2_start + y2;
				
				x3_start <= x3_start + x3;
				y3_start <= y3_start + y3;
				
				calib_counter = calib_counter + 7'd1;
				feedback_pio <= 8'd3; 
			end 
			else begin 
				x_start <= x_start >> 7; 
				y_start <= y_start >> 7;
				
				x2_start <= x2_start >> 7;
				y2_start <= y2_start >> 7;
				
				x3_start <= x3_start >> 7;
				y3_start <= y3_start >> 7;
				
				z1_start <= z1_start >> 7;
				z2_start <= z2_start >> 7;
				z3_start <= z3_start >> 7;
				
				feedback_pio <= 8'd3; 
				wrong <= 0;
				feedback_state <= 3'd1; 
				calib_counter = 7'd0; 
			end 
		end 
		if(feedback_state == 3'd1) begin
		
		 if(curr_marker == 2'd0) begin 
			z_diff <= y1 - y_start; 
		 end
		 else if(curr_marker == 2'd1) begin
			z_diff <= y2 - y2_start; 
		 end 
		 else begin
			z_diff <= y3 - y3_start; 
		 end 
		 
		 feedback_state <= 3'd2; 
		 
		end 
		
		if(feedback_state == 3'd2) begin 
		
		 if(z_diff > thres_pio ) begin //move lower
			  feedback_pio <= 8'd1;
			  wrong <= 1;
		 end
		 else if(z_diff < -thres_pio) begin //move higher
				feedback_pio <= 8'd2; 
		 end
		 else begin //dont move
			  feedback_pio <= 8'd0; 
			  wrong <= 0; 
		 end
		 
		 feedback_state <= 3'd1; 
		 
		end 
	end
 end 
end 


//=======================================================
// TOP LEVEL CONTROL
//=======================================================
reg [7:0] state;  
reg [9:0] x = 10'd0;
reg [9:0] y = 10'd0;

//wire [15:0] pixel; // SDRAM Read Data
reg [2:0] r_l = 3'd0;
reg [2:0] g_l = 3'd0;
reg [1:0] b_l = 2'd0;
reg [2:0] r_r = 3'd0;
reg [2:0] g_r = 3'd0;
reg [1:0] b_r = 2'd0;

parameter img_size = 76800;
parameter x_max = 320; 
parameter y_max = 240; 

//wire [31:0] sdram_read_data ;
reg [31:0] sdram_temp_data;  


always @(posedge CLOCK_50) begin // CLOCK_50

  if(start_signal) begin 
   // reset state machine and read/write controls
    if (~KEY[0]) begin
        vga_sram_write <= 1'b0 ; // set to on if a write operation to bus
        x <= 10'd0;
        y <= 10'd0;
        state <= 8'd0;
		  sdram_read <= 1'b1; 
		  sdram_write <= 1'b0; 
		  sdram_address <= 32'd0; 
		  //wrong <= 0; 
		  
    end 

    else begin
		 if(state == 8'd0) begin
			if(sdram_ack == 1'b1) begin 
				sdram_read <= 1'b1; 
				sdram_write <= 1'b0; 
				
				sdram_temp_data <= sdram_read_data; 
				r_l <= sdram_read_data[2:0];
				g_l <= sdram_read_data[5:3];
				b_l <= sdram_read_data[7:6];

				r_r <= sdram_read_data[10:8];
				g_r <= sdram_read_data[13:11];
				b_r <= sdram_read_data[15:14];
				
				state <= 8'd1; 
				
			end
			else begin
				state <= 8'd0; 
				sdram_read <= 1'b1; 
			end
			vga_sram_write <= 1'b0;
		 end 
		 
		 if(state == 8'd1) begin
					vga_sram_write <= 1'b1;
					vga_sram_addr <= vga_out_base_address + {22'b0, x} + ({22'b0,y}<<10);
					state <= 8'd2; 
		 end 
		 if(state == 8'd2) begin
				if(SW[0])begin
					vga_sram_writedata <= {r_l,g_l,b_l};
				end
				else if (SW[1])begin
					vga_sram_writedata <= {r_r,g_r,b_r};
				end
				else if (SW[2])begin
					vga_sram_writedata <= {r_r,g_l,b_l};
				end
				else begin
					vga_sram_writedata <= {r_l,g_r,b_r};
				end
				
			  if(x < 10'd320)begin
					x <= x + 10'd1;
			  end
			  else if (y < 10'd240) begin
					y <= y + 10'd1;
					x <= 10'd0;
			  end
			  else begin
					x <= 10'd0;
					y <= 10'd0;
			  end
			  
			  sdram_address <= {22'b0, x} +  {22'b0, y*320}; 

			  vga_sram_write <= 1'b0;
			  state <= 8'd0;
			  sdram_read <= 1'b1; 
		 end
	 end
	end 
end

//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50), //?
	.system_pll_ref_reset_reset			(1'b0),
	
	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),
	
	// bus master
	.bus_master_video_external_interface_address     (sdram_address),     // .address
	.bus_master_video_external_interface_byte_enable (4'b0011), //  .byte_enable
	.bus_master_video_external_interface_read        (sdram_read),        //   .read
	.bus_master_video_external_interface_write       (sdram_write),       //   .write
	.bus_master_video_external_interface_write_data  (sdram_write_data),  //   .write_data
	.bus_master_video_external_interface_acknowledge (sdram_ack), //    .acknowledge
	.bus_master_video_external_interface_read_data   (sdram_read_data),    //   .read_data

	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT),
	
	.onchip_vga_buffer_s1_address                    (vga_sram_addr),                    //                onchip_vga_buffer_s1.address
	.onchip_vga_buffer_s1_clken                      (vga_sram_clken),                      //                                    .clken
	.onchip_vga_buffer_s1_chipselect                 (vga_sram_chipselect),                 //                                    .chipselect
	.onchip_vga_buffer_s1_write                      (vga_sram_write),                      //                                    .write
	.onchip_vga_buffer_s1_readdata                   (vga_sram_readdata),                   //                                    .readdata
	.onchip_vga_buffer_s1_writedata                  (vga_sram_writedata),                   //                                    .writedata
	
	//pio ports
	.marker1_coords_pio_external_connection_export     (marker1_coords),
	.feedback_pio_external_connection_export			 (feedback_pio), 
	.start_signal_pio_external_connection_export     (start_signal), 
	.thres_pio_external_connection_export            (thres_pio),
	.marker2_coords_external_connection_export       (marker2_coords),       //     marker2_coords_external_connection.export
	.marker3_coords_pio_external_connection_export   (marker3_coords),
	.marker_no_pio_external_connection_export        (curr_marker)
);


endmodule
