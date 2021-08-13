/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_uart_transmitter.v
	Theme			:	Sankatmochan Bot
	Global Variables : None
	
*/

/*
Function Nmae :	main
Input 		  :	CLOCK,TX_BYTE,no_of_bytes,TX_DATA_VALID, transmit_count
Output 		  :	O_TX_SERIAL, O_TX_DONE
Logic			  :	transmits data to the zigbee using UART protocol 


Example Call  :	main utt( .CLOCK(CLOCK),
									.TX_BYTE(data),
									.TX_DATA_VALID(valid_uart),
									.O_TX_SERIAL(O_TX_SERIAL),
									.O_TX_DONE(done),
									.no_of_bytes(no_of_bytes),
									.transmit_count(transmit_count));
*/




module main(
		input CLOCK,		          	// Clock input  50 MHz
		input [95:0] TX_BYTE,			// Input Data on nets  
		input [3:0] no_of_bytes,		// number of bytes to be transmitted
		input TX_DATA_VALID,        	// make it high for atleast one clock cycle, idicates new data present on input
		output  O_TX_SERIAL,	       	// Output of tx line
		output O_TX_DONE   ,         	// high when transmission is done	
		input [2:0] transmit_count
		);

////////////////////////VARIABLES AND PARAMETERS//////////////////////////////////////

localparam	IDLE          = 3'b000;			//resets all data for transmission of next data
localparam	TX_START_BIT  = 3'b001;			//sends the start bit
localparam	TX_DATA_BIT   = 3'b010;			//sends the data bits one by one
localparam	TX_STOP_BIT   = 3'b011;			//sends the stop bit for end of transmission	
localparam	CLEANUP       = 3'b100;		
localparam	FCLEAN		  = 3'b101;
				
reg tx=1;	              //tracks output of tx line 
reg done=1;               //tracks output of O_TX_DONE	

reg [2:0] nst=IDLE;	     	// stores the next state of the programme
reg [7:0] data;         	// stores the input data locally to transmit
reg [3:0] c =0;           	// tracks the bit to be sent on output line 
reg [15:0] count=0;        // counts the duration of the bit while transmitting.
reg [9:0] duration=433;   	// 433 specific to 115200 baudrate  5208.33 for 9600
reg [15:0] fduration=8660;
reg [95:0] transdata=0;		//data to be sent

reg [3:0] nb=0;				// tracks number of byte to be sent


reg [2:0] no_times=0;		//no of times each data is to be sent (used for resolving multiple same faults at once)
 
//////////////////////////////////////////CODE/////////////////////////////////////////////


assign O_TX_SERIAL=tx;
assign O_TX_DONE=done;

always@(posedge CLOCK  )
begin
	case(nst)
		IDLE :	begin
					
					tx=1'b1;                           
					count=0;
					transdata=TX_BYTE;
					data=transdata[7:0];			//a byte of data is assigned which is to be transmitted		  
					
					if(!TX_DATA_VALID)         //when data valid high marks that no dataa is being transmitted      
					begin
					nst=IDLE;
					done=1'b1;  
					data=0;
					transdata=0;
					end
					else
					begin
						nst=TX_START_BIT;
						no_times=transmit_count;
					end
					
				end
				
				
		TX_START_BIT :	
				begin
									 
					tx=1'b0;                  
					
					done=1'b0;                
										               
					if(count>=duration)              // when necessary duration is achieved code moves to next state to transmit data
					begin
					count=0;
					nst=TX_DATA_BIT;
					end 
					else begin                       // counts from 0 to duration [to maintain needed bit duration]
					count=count+1;
					nst=TX_START_BIT;
					end
					
				end
				
		TX_DATA_BIT:
				begin
					tx=data[c];                    // initially c is 0 , c is incremented from 0 to 7 [8 bits]
					done=1'b0;
					
					if(count>=duration)             // each bit is held on dataline for necessary duration according to baudrate.
					begin
					count=0;
					nst=TX_DATA_BIT;
					
					
						if(c>6) begin                // when all data bits are transmitted then code moves to next state to transmit STOP BIT
						nst=TX_STOP_BIT;
						c=0;
						end
						else c=c+1;                 // c is incremented to transmite next bit 
						
					end 
					else begin
					count=count+1;
					nst=TX_DATA_BIT;
					end
				end
		TX_STOP_BIT:
				begin
				tx=1'b1;
				done=1'b0;
								
					if(count>=duration)             // when necessary duration is achieved code moves to next state cleanup
					begin
					count=0;
					nst=CLEANUP;
					end 
					else begin
					count=count+1;
					nst=TX_STOP_BIT;
					end
				end

		
		CLEANUP :										//resets the parameters and shifts the data to the trasmitted by 8 bits to get the next byte
				begin
					tx=1'b1;               
					if(count>=fduration)             
					begin
					
					count=0;
					
					nb=nb+1;
					
						if(nb<no_of_bytes)
						begin
							transdata=(transdata>>8);
							data=transdata[7:0];
							nst=TX_START_BIT;
						end
						else 
						begin
							no_times=no_times-1;
							nst=FCLEAN;
						end
					
					
					end 
					
					else begin
					count=count+1;
					
					nst=CLEANUP;
					end
				end
		
		FCLEAN:	
				begin
				nb=0;
				nst=IDLE;
				
				if(no_times>0)			//used for transmitting the same data multiple times (resolving multiple same colour faults)
					begin
						tx=1'b1;                           
						count=0;
						transdata=TX_BYTE;
						data=transdata[7:0];	
						nst=TX_START_BIT;
						done=0;
						
					end
				else
					begin
					done=1;
					nst=IDLE;
					end
				end
				
	endcase
end
	

endmodule
///////////////////////////////MODULE ENDS///////////////////////////
