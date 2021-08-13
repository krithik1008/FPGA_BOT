/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_uart_receiver.v
	Theme			:	Sankatmochan Bot
	Functions	:  RECEIVER	
	Global Variables : None
	
*/

/*
Function Nmae :	RECEIVER
Input 		  :	SAMP_CLOCK, O_RX_SERIAL 
Output 		  :	blocked_path,RX_DATA_DONE

Logic			  :	Gets the blocked paths details from the user through zigbee via the uart protocol 
						and creates a blocked path array in which each bit corresponds to whether or not the path is blocked, which
						is passed on to traversal module for interpretation.There are 434 samples in a singe bit at the input line and it tries
						to sample the bit from the middle of the incomming bit.
						

//connected to traversal module in the bot.bdf block diagram
);
*/
module RECEIVER(
		input SAMP_CLOCK,		          		//Clock with sampling greater than 115200
		input  O_RX_SERIAL,	         		//serial input line  mapped to TX line of Zigbee
		output reg [15:0] blocked_path,		//array indicating whoch of the path are blocked
		output RX_DATA_DONE						//indicates when trasmission is complete
		);

////////////////////////VARIABLES AND PARAMETERS//////////////////////////////////////

localparam	IDLE				= 3'b000;// samples the start bit at the middle	
localparam	RX_DATA_BIT		= 3'b001;// samples the data bit at the middle and constructs the data
localparam	RX_STOP_BIT		= 3'b010;// stop bit is sampled
localparam	INTP				= 3'b011;// interpretation of the data received
localparam	BPATH				= 3'b100;// interpretation and translation of the received data to the blocked_path array which will be passed to traversal module 
localparam	STOP				= 3'b101;
				
reg done=0;    
reg [2:0]  lcount;         	
				
initial				
blocked_path=0;

reg [2:0] next=IDLE;	       // stores the next state of the programme
reg [7:0] data;             // stores the input data locally to transmit
reg [3:0] c =0;             // tracks the bit to be input  
reg [15:0] count=0;        // counts the duration of the bit while transmitting.


reg [15:0] lprev=0; 		//for storing prev byte
reg [2:0] cdash=0;     // no. of dash


 
//////////////////////////////////////////CODE/////////////////////////////////////////////

assign RX_DATA_DONE=done;

always@(posedge SAMP_CLOCK  )
begin
	case(next)
		IDLE :	begin			// samples the start bit at the middle					
					
					if((O_RX_SERIAL == 0) && (count == 217)) 
					begin
					 done = 0;
					 next = RX_DATA_BIT;
					 count = 0;
					 c = 0;
					 end 
					 
					 else 
					 begin
						if(count>220)
							count=0;
						next=IDLE;
						count = count + 1;
					 end					                    					
					
				end
				
								
		RX_DATA_BIT:				// samples the data bit at the middle and constructs the data
				begin
				if(count==434)		// distance between the middle point of two bits is 434 samples
				begin
					data[c]=O_RX_SERIAL;                    // initially c is 0 , c is incremented from 0 to 7 [8 bits]
					done=1'b0;
					c=c+1;
					count=0;
					
					if (c==8)
						begin
						next=	RX_STOP_BIT;
						c=0;
						count=0;
						end
					else
						next=RX_DATA_BIT;
					
				end
				
				else begin
				count=count+1;
				next=RX_DATA_BIT;
				end
				
				end
		RX_STOP_BIT:					// stop bit is sampled
				begin
					if (count==434)
					begin
						if (O_RX_SERIAL == 1)
						begin
							next=INTP;
						end
						else next=IDLE;
					end
					
					else begin
					count=count+1;	
					next=RX_STOP_BIT;
					end		
				end

		
		INTP : begin					// interpretation of the data received
		
					if(data==85)		//U
					begin
						next=IDLE;
						lcount=lcount+1;
					end
				 
					else if(data==83)		//S
					begin
						next=IDLE;
						lcount=lcount+1;
					end
					
					else if(data==77)		//M
					begin
						next=IDLE;
						lcount=lcount+1;
						data=0;
					end
					
					 
					
				
					if(lcount==3)
					begin
					
						if(data==45)    //-
						begin
							cdash=cdash+1;
							
							if(cdash==2)
							begin
								next=BPATH;
								cdash=1;
							end
							else next=IDLE;
						end
						
						else
						begin
							lprev=lprev<<8;
							lprev[7:0]=data[7:0];
							next=IDLE;
						end
						
						
						
					 if(data==35)			//#
						next=STOP;
						
					end
					
				end
					
		BPATH:	begin									// interpretation and translation of the received data to the blocked_path array which will be passed to traversal module 
					case(lprev)   
					
					49:	blocked_path[1]=1'b1;
					50:   blocked_path[2]=1'b1;
					51:   blocked_path[3]=1'b1;
					52:   blocked_path[4]=1'b1;
					
					53:	blocked_path[5]=1'b1;
					54:   blocked_path[6]=1'b1;
					55:   blocked_path[7]=1'b1;
					56:   blocked_path[8]=1'b1;
					
					57:	blocked_path[9]=1'b1;
				12592:   blocked_path[10]=1'b1;
				12593:   blocked_path[11]=1'b1;
				12594:   blocked_path[12]=1'b1;
					
				12595:	blocked_path[13]=1'b1;
				12596:   blocked_path[14]=1'b1;
			   12597:   blocked_path[15]=1'b1;
					66:   blocked_path[0]=1'b1;
					
					
					endcase
					
					next=IDLE;
					lprev=0;
				
					end
		
		STOP:		begin
						next=STOP;
						done=1;
						
					end
	endcase
	
			
end
	

endmodule
///////////////////////////////MODULE ENDS///////////////////////////
