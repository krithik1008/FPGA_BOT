/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_motor.v
	Theme			:	Sankatmochan Bot
	Function		:	motor
	Global Variables : None
		
*/
	

/*

Function Nmae :	motor
Input 		  :	khz100, dirl, dirr,pwml,pwmr
Output 		  :	la,lb,ra, rb
Logic			  :	It converts the numerical pwm values in to  the actual hardware signals with the help of 100 khz clock signal. Resulting Pwm value is of 1khz 
						frequency. 

Example Call  :	motor utt2(
						 .khz100(khz100),
						 .dirl(dirl),
						 .dirr(dirr),
						 .pwml(pwml),
						 .pwmr(pwmr),
						 .la(la),
						 .lb(lb),
						 .ra(ra),
						 .rb(rb)
						 );
*/






module motor(
input khz100,				// 100khz clock from pll 
input dirl,					// 1 for forward 0 for backward
input dirr,					// 1 for forward 0 for backward
input [7:0] pwml,			// pwm value of left motor
input [7:0] pwmr,			// pwm value of right motor

output reg la,				// Hardware signal for left motor
output reg lb,				// Hardware signal for left motor 
output reg ra,				// Hardware signal for right motor
output reg rb				// Hardware signal for right motor
);

reg [7:0] count=0;
reg levell;
reg levelr;
 
always @(posedge khz100) 		// forms the pwm signal ... 100 counts of 100 khz signal = 1 milli sec period 
begin
count=count+1;

		if (count>100)
		count =0;

		if(count <pwml)
		levell=1;
		else levell=0;


		if(count <pwmr)
		levelr=1;
		else levelr=0;
		
end



always @ (posedge khz100)		// assigns the pwm signal according to the direction required to the hardware pins
begin

	if(dirl==1)
	begin
		la=levell;
		lb=0;
	end
	else begin
		lb=levell;
		la=0;
	end

	if(dirr==1)
	begin
		ra=levelr;
		rb=0;
	end
	else begin
		rb=levelr;
		ra=0;
	end
	
end

 

endmodule

