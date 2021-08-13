/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_ADC_controllerr.v
	Theme			:	Sankatmochan Bot
	Function		: 	ADC_controller
	Global Variables : None
	
*/

/*
Function Nmae :	ADC_controller
Input 		  :	sclk,dout,rst
Output 		  :	ADC_DATA_CH1,ADC_DATA_CH2,ADC_DATA_CH3

Logic			  :	Converts serial data from the adc into parallel data. Used for 
						getting line sensor values (left, right and center channel values)


Example Call  :	ADC_controller utt1(
						.sclk(mhz32),			
						.dout(adcin),
						.rst(0),
						.cs(cs),
						.din(din),
						.ADC_DATA_CH1(right),
						.ADC_DATA_CH2(center),
						.ADC_DATA_CH3(left)
);
*/





module ADC_controller

(sclk,dout,rst,cs,din,
ADC_DATA_CH1,
ADC_DATA_CH2,
ADC_DATA_CH3);

input sclk,dout,rst;
output reg [11:0] ADC_DATA_CH1,ADC_DATA_CH2,ADC_DATA_CH3;		//output data lines

output reg cs,din;

reg [3:0] count; 								//for keeping track of clock cycles
reg [2:0] add_count=3'b000;				//for storing which address cycle it is in
//we are using pins 2,7,5 as it was convinient for wiring
reg [2:0] analog_address1=3'b010;		//address of analog pin 2
reg [2:0] analog_address2=3'b111;		//address of analog pin 7
reg [2:0] analog_address3=3'b101; 		//address of analog pin 5

reg [2:0] address=3'b010;					//address to be sent (initially pin 2)

reg [11:0] temp_data=12'b0;				//temporary register for storing the channel output

initial
begin
	count=4'b0000;
	ADC_DATA_CH1=12'b0;						//initialising all output channels to 0
	ADC_DATA_CH2=12'b0;
	ADC_DATA_CH3=12'b0;
	cs=1'b1;										//chip select is initially high
end


always@(posedge sclk or posedge rst)
begin
	cs<=0;
	if(rst==1)
		begin
		count <= 4'b0000;						//making chip select high when reset is triggered and clk count is reset to 0
		end
	else if(cs==0)
		count <= count + 4'b0001;			//counting the clock cycles
end

always@(negedge sclk or posedge rst)	//to send address to the ADC in 3 clock cycles 
begin
	if(rst==1)
	begin
	add_count<=3'b000;						//resetting address when reset is triggered
	din<=0;
	//cs<=1'b1;
	end
	else
	begin									
	case(count)									//sending address bit by bit via din
		4'b0001:	din <= address[2];
		4'b0010:	din <= address[1];
		4'b0011:	din <= address[0];
		4'b0100:	begin
						din<=0;							//Updating address count and logic for sending address cyclically
						if(add_count==3'b000)
							address<=analog_address2;
						if(add_count==3'b001)
							address<=analog_address3;
						if(add_count==3'b010)
							address<=analog_address1;
						
						if(add_count==3'b010)
							add_count<=3'b000;
						else 
							add_count<=add_count+3'b001;
							
					end
		default: din=0;
	endcase
	end
end

always@(posedge sclk or posedge rst)   //collecting the serial data bits and outputting in ADC_CHx in the 16th cycle
begin
	if(rst==1)									//reset opeartion sets all output channels to 0
	begin
	ADC_DATA_CH1<=12'b0;
	ADC_DATA_CH2<=12'b0;
	ADC_DATA_CH3<=12'b0;
	end
	else
	begin
	case(count)								
		4'b0000: begin
					if(add_count==3'b000) 	//whatever the current address, the ADC outputs the data it received in the previous cycle's address
						ADC_DATA_CH2 <= temp_data;	
					else if(add_count==3'b001)
						ADC_DATA_CH3 <= temp_data;	
					else if(add_count==3'b010)
						ADC_DATA_CH1 <= temp_data;	
					end
		//converting serial to parallel data
		4'b0011: temp_data[11] <= dout;		//assigning bits as they come, to the temporary register,
		4'b0100: temp_data[10] <= dout;		
		4'b0101: temp_data[9] <= dout;
		4'b0110: temp_data[8] <= dout;
		4'b0111: temp_data[7] <= dout;
		4'b1000: temp_data[6] <= dout;
		4'b1001: temp_data[5] <= dout;
		4'b1010: temp_data[4] <= dout;
		4'b1011: temp_data[3] <= dout;
		4'b1100: temp_data[2] <= dout;
		4'b1101: temp_data[1] <= dout;
		4'b1110:	temp_data[0] <= dout;
	endcase
	end
end


endmodule
