/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_colour_sensor.v
	Theme			:	Sankatmochan Bot
	Global Variables : None
	
*/

/*

Function Nmae :	colour_sensor
Input 		  :	sensor, measure, clk
Output 		  :	S3,color,valid
Logic			  :	Detects the colour patches using the colour sensor and return the colour to the message module 

Example Call  :	colour_sensor utt1( 	.sensor(sensor),
													.measure(measure),
													.clk(CLOCK),
													.S3(S3),
													.color(color),
													.valid(valid_color));
*/





module colour_sensor(
input sensor,        			// sensor input 
input measure,						// to start measurement
input clk,	   					// 50mhz input clock
output reg S3,						//to be mapped to fpga to change filter
output reg S2,						//to be mapped to fpga to change filter
output reg [2:0] color, 		//result
output reg valid 					//becomes high when result is available 
);

//states of state diagram

localparam	IDLE       = 3'b000;			//idle state
localparam  WHITE		  = 3'b001;
localparam	RED	     = 3'b010;			//state when red filter is active
localparam	BLUE	     = 3'b011;			//state when blue filter is active 
localparam	DONE       = 3'b100;			//completion of measurement 
 


reg [19:0] count=0;					//to count positive edges and comapre with duration 
reg [19:0] duration=600000;      // duation of count we are measuring the positive edge 
reg [15:0] pulse=0;					// count the time a pulse is high ( no logic is driven form this)
reg [9:0] poscount=0;           // counts the positive edge which will decide the color

 
reg [7:0] WTH=65;					//threshold for no color patch (if poscount>65 then it is white (no patch))
reg [7:0] UTH=40;					//threshold for red/blue (if poscount>40 and poscount<65 then red/blue patch) depending on filter

reg [2:0] nxt=IDLE;

always@(posedge clk)
begin

	case(nxt)
		IDLE:			begin			//idle case where everything is initialized to 0
							count=0;
							pulse=0;
							valid=0;
							poscount=0;
							nxt=WHITE;
							S2=1;
							S3=0;
						end
						
						
		WHITE:		begin
							
							
							count=count+1;
							if(count<duration)
							begin
								if(sensor)			//if sensor output is high then increase pulse 
									pulse=pulse+1;
								else
								begin
									if(pulse>0)
									begin
										pulse=0;
										poscount=poscount+1; //counting no. of posedges of pulse to compare with thresholds
									end
								end						
							end
							
							else 
							begin
									if(poscount>100)       //Comparing poscount with threshold for RED color
									begin
										color=3'b000;      		//assigning RED color value 
										count=0;
										nxt=DONE;
									end
			
									else
									begin	
											nxt=RED;				//If neither is satisfied going to blue filter state
											S3=0;                // for setting red filter
											S2=0;
											count=0;
											pulse=0;
											poscount=0;
									end
							end
							
						
						
						end
						
		RED:			begin							//red filter is active
						
							count=count+1;
							if(count<duration)
							begin
								if(sensor)			//if sensor output is high then increase pulse 
									pulse=pulse+1;
								else
								begin
									if(pulse>0)
									begin
										pulse=0;
										poscount=poscount+1; //counting no. of posedges of pulse to compare with thresholds
									end
								end						
							end
							
							else 
							begin
									if(poscount>45)       //Comparing poscount with threshold for RED color
									begin
										color=3'b001;      		//assigning RED color value
										nxt=DONE;					//as red is detected in only 1 cycle (2 cycles for blue/green), making it wait for 1 cycle for uniformity 
										count=0;
									end
									
									else
									begin	
											nxt=BLUE;				//If neither is satisfied going to blue filter state
											S3=1;                      // for setting blue filter
											S2=0;
											count=0;
											pulse=0;
											poscount=0;
									end
							end
							
						end
		
		BLUE:			begin									//blur filter is active
					
							count=count+1;
							if(count<duration)
							begin
								if(sensor)					//if sensor output is high then increase pulse
									pulse=pulse+1;
								else
								begin
									if(pulse>0)
									begin
										pulse=0;
										poscount=poscount+1;		//counting no. of posedges of pulse to compare with thresholds
									end
								end						
							end
							
							else 
							begin
									if(poscount>=UTH)       //Comparing poscount with threshold for BLUE color
									begin
										color=3'b011;     	//assigning BLUE color value
										nxt=DONE;
									end
									
									else						//If the previous conditions fail then the patch is green color
									begin	
										color=3'b010;     // assigning GREEN color value
										nxt=DONE;
									end
							end
							
						end
						
						
		DONE:			begin							//measurement is complete and result is ready for other modules to use
							valid=1;
							if (measure==0)
							nxt=DONE;
							else 
							nxt=IDLE;
						end
		

	
	endcase
end

endmodule 