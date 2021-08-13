/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_12C_Controller.v
	Theme			:	Sankatmochan Bot
	Function		:	I2C_Controller
	Global Variables : None
		
*/
	

/*

Function Nmae :	I2C_Controller
Input 		  :	clk , rst , core_busy
Output 		  :	data_valid, rw , slave_addr, reg_addr, reg_data	

Logic			  :	With the help of state diagram different data is passed to I2C_core module which implements the I2C protocol. It sets the measure bit in the accelereometer regesters
						and then read the x axis data from the same. 
						
Example Call  :	module I2C_Controller( .clk(clk),          
													  .rst(rst),         
													  .core_busy(core_busy),    

													 .data_valid(data_valid),   
													 .rw(rw),				
													 .slave_addr(slave_addr),	
													 .reg_addr(reg_addr),		
													 .reg_data(reg_data)		
													);

*/



// assuming PIN-12(ALT-ADDRESS) of the ADXL345 IS GROUNDED = 0x53 is the address of the adxl345 sensor on-board
// CS of core is connected to CS' (pin 7) OF ADXL345 i.e I2C IS ENABLED ON THE CHIP 
// 100hz default speed of communication on ADXL345
// PWR-CTL Regester is 0x00 by default which puts the sensor in standby mode , thus measurement bit need to be set. 
// we are performing two write operation and two read operation. 
// 1st write to set the necessary bits in resolution regester.
// 2nd write to set the measurement bit in the PWR-CTL regester so that sensor initiates its measuremt.
// 2 read operation on 0x32 address to obtain x axis data.

// 0x2D POWER_CTL   =8'b00001000  (measurement bit high)
// 0x31 DATA_FORMAT =8'b00000000  ( justify bit=0 LSB mode with  +/- 2g 10 bit resolution)



// one is to program again and again 
// one is to directly start the oeration of adxl 


module I2C_Controller(

input  clk,                   // 50MHz 
input  rst,                   // common to both the core and control 
input  core_busy,             // input from core 

output reg data_valid,        // output to core
output reg rw,						// output to core
output reg [6:0] slave_addr,	// output to core
output reg [7:0] reg_addr,		// output to core
output reg [7:0] reg_data		// output to core
);

//////////////////////////VARIABLES AND PARAMETERS////////////////////

localparam	IDLE            = 1;
localparam 	BAND				 = 2;
localparam	WAIT_1_CYCLE_1  = 3;
localparam 	READ_REG			 = 4;
localparam 	SET_RES			 = 5;
localparam	WAIT_1_CYCLE_2  = 6;
localparam 	READ_REG_2		 = 7;
localparam	START_OPERATION = 8;
localparam	WAIT_1_CYCLE_3  = 9;
localparam	READ_DATA       = 10;



reg [7:0] nst =IDLE;        // tracks the next state of the programme
reg [25:0] count =0;


/////////////////////////////////CODE/////////////////////////////////

always@(posedge clk,negedge rst)
begin
	if (rst==0) nst=IDLE;
	else begin
	case (nst)
		IDLE :
				begin
				data_valid<=1'b0;
				slave_addr<=7'b0000000;
				reg_addr  <=8'b00000000;
				reg_data  <=8'b00000000;
				
				if(rst) nst<=BAND;
				else nst<=IDLE;				
				end
				
		BAND:                           // sets the POWER_CTL regester
				begin								
					if (!core_busy)
					begin
						rw        <=0;                  // write operation
						slave_addr<=7'h1D;
						reg_addr  <=8'h2C;              // reg address
						reg_data  <=8'h05;              // sets the measurement bit
						data_valid<=1'b1;               // send the data
						nst<=WAIT_1_CYCLE_1;
					end
					else nst<=BAND;				
				end
		
		WAIT_1_CYCLE_1:
				begin
						
				data_valid<=1'b0; 
					if (!core_busy)
						nst<=READ_REG;
					else
						nst<=WAIT_1_CYCLE_1;			
				end
				
		READ_REG:
				begin		
					if (!core_busy)
						begin
						rw        <=1;                  // read operation
						slave_addr<=7'h1D;
						reg_addr  <=8'h2C;              // x-axis data regester in ADXL345 , if we make subsequent read operation, it will give data present at next address of reg 
						reg_data  <=8'h00;
						data_valid<=1'b1;               // send the data
						end
					else begin
					data_valid<=1'b0;
					end
					nst<=SET_RES;
				end
				
				
		SET_RES:                           // sets the POWER_CTL regester
				begin								
					if (!core_busy)
					begin
						rw        <=0;                  // write operation
						slave_addr<=7'h1D;
						reg_addr  <=8'h31;              // reg address
						reg_data  <=8'h02;              // sets the measurement bit
						data_valid<=1'b1;               // send the data
						nst<=WAIT_1_CYCLE_2;
					end
					else nst<=SET_RES;				
				end
		
		WAIT_1_CYCLE_2:
				begin
						
				data_valid<=1'b0; 
					if (!core_busy)
						nst<=READ_REG_2;
					else
						nst<=WAIT_1_CYCLE_2;			
				end
				
		READ_REG_2:
				begin		
					if (!core_busy)
						begin
						rw        <=1;                  // read operation
						slave_addr<=7'h1D;
						reg_addr  <=8'h2C;              // x-axis data regester in ADXL345 , if we make subsequent read operation, it will give data present at next address of reg 
						reg_data  <=8'h00;
						data_valid<=1'b1;               // send the data
						end
					else begin
					data_valid<=1'b0;
					end
					nst<=START_OPERATION;
				end
				
	START_OPERATION:                           // sets the POWER_CTL regester
				begin								
					if (!core_busy)
					begin
						rw        <=0;                  // write operation
						slave_addr<=7'h1D;
						reg_addr  <=8'h2D;              // reg address
						reg_data  <=8'h08;              // sets the measurement bit
						data_valid<=1'b1;               // send the data
						nst<=WAIT_1_CYCLE_3;
					end
					else nst<=START_OPERATION;				
				end
		
	WAIT_1_CYCLE_3:
				begin
						
				data_valid<=1'b0; 
					if (!core_busy)
						nst<=READ_DATA;
					else
						nst<=WAIT_1_CYCLE_3;			
				end
				
		READ_DATA:
				begin		
					if (!core_busy)
						begin
						rw        <=1;                  // read operation
						slave_addr<=7'h1D;
						reg_addr  <=8'h32;              // x-axis data regester in ADXL345 , if we make subsequent read operation, it will give data present at next address of reg 
						reg_data  <=8'h00;
						data_valid<=1'b1;               // send the data
						end
					else begin
					data_valid<=1'b0;
					end
					nst<=READ_DATA;
				end
		
	endcase
end

end

endmodule
