/*
	Team ID		:	SM#1280
	Author list	: 	Krithik sankar, Ayush Mittal
	Filename		: 	SM_1280_traversal.v
	Theme			:	Sankatmochan Bot
	Functions	:  Traversal
	Global Variables : None
	
*/
	
	
/*
	Function Name		:	Traversal	
	States				:	path_block, start, measure, cleft, cright, fleft, fright, decision,
								uturn, calculation_mpu, calculation_ssu, calculation_pu, calculation_w,
								scan_traversal, scan_init, luturn, unit_path_init, idle, s1, s2, s3,
								turn_delay, run_end
	
	
	General Comment	:	It implements the line following algorithm (measure, cleft, cright, fleft, fright), calculates the entry node and exit node of each manufacturing 
								unit after receiving the blocked path list from the message unit(calculation_mpu, calculation_ssu, calculation_pu, calculation_w,),it also implements 
								the dijkstra algorithm with the help of idle,s1,s2,s3 states , along with that it translates the path mapped out by the dijkstra algorithm in to actual mechanical motions 
								with the help of decision state.it also autocorrects the orientation of the bot after it reaches one of the units to scan for faults.
								
	Logic					:  It first wait for the bot to receive the blocked path in the path_block state. It then moves to the start state waiting for the key0 to be pressed , once, the key is 
								pressed calculations are made to speculate the path ,entry node of each manufacturing unit in such a way that path from entry to exit node is in clockwise direction.
								We have installed the color sensor on the right side of the bot therefore scanning the units in the clockwise direction will always result in the sensor to be on the side where the patches are present.
								After all the calculations are made, dijkstra algorithm is executed to calculate the path from current node to the entry node of MPU or PU.Here the trick is that source node is 
								initialised to be the entry node of the unit rather than the current node of the bot , this helps us to read the path to be followed directly from the path array in a straight forward fashion rather than by conventional backtracking the 
								the destination point. After the path is ready then the bot moves to the decision state where we have mapped each node in to an individual entity which translates the path from the path array to the actual motions 
								depending upon the previous node , current node and the next immediate node. Once the bot reaches the entry point of any unit then the code moves to the scan_init state where the path array is re-initialized 
								according to calculations made at the starting of the traversal to begin with scanning of the unit, along with that the bot also choose the entry point of the next unit it has to go after finishing up the scanning. 
								It then corrects its orientation with the help of decision state and after that it scan the unit with the help of scan_traversal,cleft,and cright states.Once it reaches the exit node of the 
								unit, again dijkstra is implemented to find the way to the SSU unit. After reaching the ssu unit and scaning the same from the logic mentioned above, the code again follows the same logic described above untill the variables pu_count,
								mpu_count and w_count becomes zero. These variables are decremented each time the bot scans the corresponding unit , and are incremented in the case of fire or when the faults are more then 3.After scanning a unit , 
								the bot will traverse towards the SSU unit irrespective of the number of faults it is carrying.
								
								
							
								
	Example Call	:	   It is the central module where all other modules are called.
	

	
*/
module traversal(
	//adc variables (line follower)
	adcin,			//dout from the adc module 
	cs,
	mhz32,			//adc runs with 3.2 MHz clock
	din,
	
	//motor input output variables
	khz100,			//using 100kHz clock to run motor at 1KHz
	la,				//left motor
	lb,
	ra,				//right motor
	rb,
	
	motor_3v,		//using a GPIO pin for 3.3v ref of motor driver
	  
	//node tansmission variables
	node_detect,
	node_count,
	
	//for staring traversal when key 0 is pressed 
	key0,
	
	// accelerometer variables
	accel,      // value from accelerometer  
	accelvalid,
	
	//uart receiver
	blocked_path,				// number of bytes to be transmitted
	RX_DATA_DONE,
	
	//message variables
	in_scan,
	unit,
	is_stop,
	pu_count,
	mpu_count,
	w_count,
	red_flagg
);
// red detection 
input red_flagg;

//uart receiver
input  [15:0] blocked_path; // array which tells which path is blocked 0th index tells the state of bridge path, other indexes conveys that the corresponding path is blocked if the value is 1.
input RX_DATA_DONE;			 // if one then data is ready in blocked_path

// acceleromerter variables 
input [15:0] accel ;
reg [31:0] accel_delay=0;			//a small amount of delay has been introduced so that bot can reach to the slope of bridge 
output reg accelvalid=1;											
										 
//when node is detected these variables helps in conveying this to message module 
output reg node_detect=0;			//1 if a node is detected
output reg [5:0] node_count=0;	
reg node=0;								//temp variable for node detection 
  

// key0 implementation
input key0;

//delay variables
reg [23:0] wait_count=0;			// random delay variabe
reg [23:0] ddelay=490000; 			// bot detects the node in the very beginnig, because of which turning left/right to the next path becomes difficult, 
											// to overcome that we have introdoced a small delay so that the bot crosses the node first after detection and then execute the turn left / turn right protocol
reg [23:0] slope_count=0;			// sometimes, the bot completely stops while descending down the bridge because ,reverse braking is executed continously , and the speed of the bot reduces to 0 on the slope itself
											// this variable is used to prevent that state. 

//adc variables
input adcin;
input mhz32;			//adc runs with 3.2 MHz clock
output cs;				//chip select
output din;
wire [11:0] center;	//center sensor values
wire [11:0] left;		//left sensor values
wire [11:0] right;	//right sensor values



//adc instantiation for line following
ADC_controller utt1(
	.sclk(mhz32),			
	.dout(adcin),
	.rst(0),
	.cs(cs),
	.din(din),
	.ADC_DATA_CH1(right),
	.ADC_DATA_CH2(center),
	.ADC_DATA_CH3(left)
);

//motor variables
reg [7:0] turn_pwn	=	37;
reg [7:0] turn_pwn_r	=	37;

reg [7:0] base_pwm 	= 	39;	//base speed (pwm) of motors 
reg [7:0] offset1 	= 	37;		//base will be increased by offest1 
reg [7:0] offset2		= 	39;		//base will be decreased by offest2 
reg [7:0] pwml=0;          //pwm value of left motor (input to motor module)
reg [7:0] pwmr=0;			   //pwm value of right motor (input to motor module)
reg dirl=1'b1;				   //direction of motor (forward/backward), 1 for forward
reg dirr=1'b1;				 

input khz100;				//100KHz clock to make it 1KHz for motor
output la,lb,ra,rb;

output motor_3v=1;		//using a GPIO pin for 3.3v ref of motor driver

//motor instantiation
motor utt2(
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
 
//message variables
output reg in_scan=0;			//becomes 1 if bot in scanning any one the units
output reg [2:0] unit = 0;		//indicates the unit no. to the message module
										//0:no unit
										//1:MPU unit
										//2:PU unit
										//3:W unit
										//4:SSU unit
output reg  is_stop=0;			//bwcomes 1 at the end of run
							

//calculation parameter for deciding pwm of motors
reg [11:0] center_left;				//absolute value of (center - left)
reg [11:0] center_right;			//absolute value of (center - right)
reg [6:0] range = 20;				//value for deciding when to actuate motors for path correction 

//state diagram for traversal
localparam path_block	= 5'b00000;			//for indicating the blocked paths by updating the adjacency matrix
localparam start			= 5'b00001;			//starts motion of the bot when key 0 is pressed
localparam measure		= 5'b00010;			//compare parameter with range to decide correction
localparam cleft			= 5'b00011;			//left motor will actuate (move right)	to correct path
localparam cright			= 5'b00100;			//right motor will actuate (move left)	to correct path
localparam fleft			= 5'b00101;			//full left state : to turn left on nodes
localparam fright			= 5'b00110;			//full right state : to turn right on nodes
localparam decision		= 5'b00111;			//decides what action (left,right,etc) to take depending on the path matrix 
localparam uturn	 		= 5'b01000;			//when a node is detected , this state tells the next course of action
localparam calculation_mpu	=5'b01001;		////These states calculates the entry node, exit nodeand	
localparam calculation_ssu	=5'b01010;
localparam calculation_pu	=5'b01011;
localparam calculation_w	=5'b01100;
localparam scan_traversal	=5'b01101;	//same as measure state , but helps in traversing while the bot is scanning the manufacturing unit	
localparam scan_init			=5'b01110;	// appends the path matrix with the path of UNIT that needs to be traversed, also decides the next unit needs to be scanned
localparam luturn	 = 5'b01111;			//to take 180 degree turn , mainly used to correct orientation (left side uturn)
localparam unit_path_init	=5'b10000;	// path matrix of each individual unit are initialised to a default value in this state

// states for dijkstra algorithm 
localparam	idle	= 5'b10001; 
localparam	s1		= 5'b10010;	// finds the minimum in the distance matrix
localparam	s2		= 5'b10011;	// implements the algorithm 
localparam  s3		= 5'b10100;	//	takes care of the no. of iterations the sates has to be executed to fully calculate the path 


localparam turn_delay	=	5'b10101;	// implements a delay for the nodes in which turns are need to be taken so that the bot crosses the node first and then turn protocol is executed. it is not
localparam run_end	=	5'b10110;


reg [4:0] nst	=	start; 	//start;
reg [4:0] temp_state;				

reg [4:0] prev	=	scan_traversal;

// state diagram of traversing the graph
// variables assigned to each physical and the virtual node. 
// Impoertant note : we have introduced two virtual node in our algorithm for PU unit and W unit to reduce the algorithm complexity.
localparam n1   = 0;
localparam n2   = 1;			
localparam n3   = 2;
localparam n4   = 3;		
localparam n5   = 4;			
localparam n6   = 5;	
localparam n7   = 6;
localparam n8   = 7;
localparam n9   = 8;
localparam n10  = 9;
localparam n11  = 10;
localparam n12  = 11;
localparam n13  = 12;
localparam n14  = 13;


//node state varibles 

reg [7:0] prevn	=	n1;
reg [7:0] curn		=	n1;
reg [7:0] nxtn;


// no. of nodes it needs to traverse while scanning a mnufacturing unit
reg [7:0] node_mpu;
reg [7:0] node_pu;
reg [7:0] node_ssu;
reg [7:0] node_w;


reg [7:0] path_mpu [0:13];
reg [7:0] path_w [0:13];
reg [7:0] path_pu [0:13];
reg [7:0] path_ssu [0:13];

//entry nodes of each unit
 reg [7:0] en_mpu=0;
 reg [7:0] en_pu=0;
 reg [7:0] en_ssu=0;
 reg [7:0] en_w=0;

 // temperorary variables used in calculation states
 reg  a=1;
 reg  b=1;
 reg  c=1; //for path blocking during scanning

//number of times bot has to visit each unit
input [7:0] pu_count;
input [7:0] mpu_count;
input [7:0] w_count;	

/////////////////dijkstra variables////////////////////
reg [7:0] pos=6'b111111;

// loop variables used in idle,s1,s2,s3 state 
reg [3:0] i	=	0;
reg [3:0] j	=	0;
reg [3:0] k	=	0;	
reg [3:0] x =  0;

reg [7:0] adj [0:13][0:13];	// adjacency matrix

reg [7:0] dijk_count=0;

reg [7:0] s_node=0;            // source node
reg [7:0] dist [0:13];			// distance array
reg [1:0] visited [0:13];		// visited array 
reg [7:0] path [0:13];			// path array , where the path to be traversed is stored 


//variables for min state 
reg [7:0] temp=255;

initial begin
	
	adj[0][0]=0;
	adj[0][1]=1;
	adj[0][2]=64;
	adj[0][3]=64;
	adj[0][4]=64;
	adj[0][5]=64;
	adj[0][6]=64;
	adj[0][7]=64;
	adj[0][8]=64;
	adj[0][9]=64;
	adj[0][10]=64;
	adj[0][11]=64;
	adj[0][12]=64;
	adj[0][13]=64;
	
	
	adj[1][0]=1;
	adj[1][1]=0;
	adj[1][2]=1;
	adj[1][3]=64;
	adj[1][4]=64;
	adj[1][5]=64;
	adj[1][6]=64;
	adj[1][7]=64;
	adj[1][8]=64;
	adj[1][9]=64;
	adj[1][10]=64;
	adj[1][11]=64;
	adj[1][12]=64;
	adj[1][13]=1;
	
	adj[2][0]=64;
	adj[2][1]=1;
	adj[2][2]=0;
	adj[2][3]=1;
	adj[2][4]=1;
	adj[2][5]=64;
	adj[2][6]=64;
	adj[2][7]=64;
	adj[2][8]=64;
	adj[2][9]=64;
	adj[2][10]=64;
	adj[2][11]=64;
	adj[2][12]=64;
	adj[2][13]=64;
	
	adj[3][0]=64;
	adj[3][1]=64;
	adj[3][2]=1;
	adj[3][3]=0;
	adj[3][4]=64;
	adj[3][5]=1;
	adj[3][6]=64;
	adj[3][7]=64;
	adj[3][8]=64;
	adj[3][9]=64;
	adj[3][10]=64;
	adj[3][11]=64;
	adj[3][12]=64;
	adj[3][13]=64;
	
	adj[4][0]=64;
	adj[4][1]=64;
	adj[4][2]=1;
	adj[4][3]=64;
	adj[4][4]=0;
	adj[4][5]=1;
	adj[4][6]=64;
	adj[4][7]=64;
	adj[4][8]=64;
	adj[4][9]=64;
	adj[4][10]=64;
	adj[4][11]=64;
	adj[4][12]=64;
	adj[4][13]=64;
	
	adj[5][0]=64;
	adj[5][1]=64;
	adj[5][2]=64;
	adj[5][3]=1;
	adj[5][4]=1;
	adj[5][5]=0;
	adj[5][6]=1;
	adj[5][7]=1;
	adj[5][8]=64;
	adj[5][9]=64;
	adj[5][10]=64;
	adj[5][11]=64;
	adj[5][12]=64;
	adj[5][13]=64;
	
	adj[6][0]=64;
	adj[6][1]=64;
	adj[6][2]=64;
	adj[6][3]=64;
	adj[6][4]=64;
	adj[6][5]=1;
	adj[6][6]=0;
	adj[6][7]=1;
	adj[6][8]=64;
	adj[6][9]=64;
	adj[6][10]=64;
	adj[6][11]=64;
	adj[6][12]=1;
	adj[6][13]=64;
	
	adj[7][0]=64;
	adj[7][1]=64;
	adj[7][2]=64;
	adj[7][3]=64;
	adj[7][4]=64;
	adj[7][5]=1;
	adj[7][6]=1;
	adj[7][7]=0;
	adj[7][8]=1;
	adj[7][9]=64;
	adj[7][10]=64;
	adj[7][11]=64;
	adj[7][12]=64;
	adj[7][13]=64;
	
	adj[8][0]=64;
	adj[8][1]=64;
	adj[8][2]=64;
	adj[8][3]=64;
	adj[8][4]=64;
	adj[8][5]=64;
	adj[8][6]=64;
	adj[8][7]=1;
	adj[8][8]=0;
	adj[8][9]=1;
	adj[8][10]=1;
	adj[8][11]=64;
	adj[8][12]=64;
	adj[8][13]=64;
	
	adj[9][0]=64;
	adj[9][1]=64;
	adj[9][2]=64;
	adj[9][3]=64;
	adj[9][4]=64;
	adj[9][5]=64;
	adj[9][6]=64;
	adj[9][7]=64;
	adj[9][8]=1;
	adj[9][9]=0;
	adj[9][10]=64;
	adj[9][11]=1;
	adj[9][12]=64;
	adj[9][13]=64;
	
	adj[10][0]=64;
	adj[10][1]=64;
	adj[10][2]=64;
	adj[10][3]=64;
	adj[10][4]=64;
	adj[10][5]=64;
	adj[10][6]=64;
	adj[10][7]=64;
	adj[10][8]=1;
	adj[10][9]=64;
	adj[10][10]=0;
	adj[10][11]=1;
	adj[10][12]=64;
	adj[10][13]=64;
	
	adj[11][0]=64;
	adj[11][1]=64;
	adj[11][2]=64;
	adj[11][3]=64;
	adj[11][4]=64;
	adj[11][5]=64;
	adj[11][6]=64;
	adj[11][7]=64;
	adj[11][8]=64;
	adj[11][9]=1;
	adj[11][10]=1;
	adj[11][11]=0;
	adj[11][12]=1;
	adj[11][13]=1;
	
	adj[12][0]=64;
	adj[12][1]=64;
	adj[12][2]=64;
	adj[12][3]=64;
	adj[12][4]=64;
	adj[12][5]=64;
	adj[12][6]=64;
	adj[12][7]=64;
	adj[12][8]=64;
	adj[12][9]=64;
	adj[12][10]=64;
	adj[12][11]=1;
	adj[12][12]=0;
	adj[12][13]=1;
	
	adj[13][0]=64;
	adj[13][1]=1;
	adj[13][2]=64;
	adj[13][3]=64;
	adj[13][4]=64;
	adj[13][5]=64;
	adj[13][6]=64;
	adj[13][7]=64;
	adj[13][8]=64;
	adj[13][9]=64;
	adj[13][10]=64;
	adj[13][11]=1;
	adj[13][12]=1;
	adj[13][13]=0;
end
	
always@(posedge mhz32)		//3.2 MHz clock
begin

	case(nst)
	path_block:		begin
						if(RX_DATA_DONE==0)
							nst=path_block;
							
						else if(RX_DATA_DONE==1)
						begin
							if(blocked_path[0]==1)
							begin
								adj[6][12]=64;
								adj[12][6]=64;
							end
							if(blocked_path[1]==1)
							begin
								adj[13][1]=64;
								adj[1][13]=64;
							end
							if(blocked_path[2]==1)
							begin
								adj[11][13]=64;
								adj[13][11]=64;
							end
							if(blocked_path[3]==1)
							begin
								adj[11][10]=64;
								adj[10][11]=64;
								adj[9][11]=64;
								adj[11][9]=64;
							end
							if(blocked_path[4]==1)
							begin
								adj[9][8]=64;
								adj[8][9]=64;
								ddelay=350000;
							end
							if(blocked_path[5]==1)
							begin
								adj[7][8]=64;
								adj[8][7]=64;
							end
							if(blocked_path[6]==1)
							begin
								adj[7][5]=64;
								adj[5][7]=64;
							end
							if(blocked_path[7]==1)
							begin
								adj[5][4]=64;
								adj[4][5]=64;
								adj[3][5]=64;
								adj[5][3]=64;
							end
							if(blocked_path[8]==1)
							begin
								adj[2][4]=64;
								adj[4][2]=64;
							end
							if(blocked_path[9]==1)
							begin
								adj[1][2]=64;
								adj[2][1]=64;
							end
							if(blocked_path[10]==1)
							begin
								adj[12][13]=64;
								adj[13][12]=64;
								ddelay=300000;
							end
							if(blocked_path[11]==1)
							begin
								adj[11][12]=64;
								adj[12][11]=64;
								ddelay=300000;
							end
							if(blocked_path[12]==1)
							begin
								adj[8][10]=64;
								adj[10][8]=64;
								ddelay=450000;
							end
							if(blocked_path[13]==1)
							begin
								adj[7][6]=64;
								adj[6][7]=64;
							end
							if(blocked_path[14]==1)
							begin
								adj[6][5]=64;
								adj[5][6]=64;
							end
							if(blocked_path[15]==1)
							begin
								adj[2][3]=64;
								adj[3][2]=64;
							end
							
							nst=start;
						end
						
						
						end
	
	start:	begin
					if (!key0)			//traversal will only begin when key 0 is pressed 
					nst=unit_path_init;
					else nst=start;
				end
				
	unit_path_init:	
				begin
					for (x=0;x<14;x=x+1)
					begin
						path_mpu[x]	= -1;
						path_w[x]	= -1;
						path_pu[x]	= -1;
						path_ssu[x] = -1;
					end
					nst=calculation_mpu;			//calculation_mpu
				end
// entry node , no. of nodes to be traversed and the path matrix of the units 
				//are calculated and initialized in the next 4 states				
	calculation_mpu:
				begin
						if(adj[13][11]==64)
							a=0;
						else a=1;
						
						if(adj[11][12]==64)
							b=0;
						else b=1;
						
						if(adj[12][13]==64)
							c=0;
						else c=1;
							
						if((a&b&c)==1)
							begin
								node_mpu=3;
								en_mpu=n14;
								
								path_mpu[n14]=n12;
								path_mpu[n12]=n13;
								path_mpu[n13]=n14;
							end
						else if((a&b&~c)==1)
							begin
								node_mpu=2;
								en_mpu=n14;
			
								
								path_mpu[n14]=n12;
								path_mpu[n12]=n13;
								path_mpu[n13]=n13;
							end
						else if((a&~b&~c)==1)
							begin
								node_mpu=1;
								en_mpu=n14;
								
								path_mpu[n14]=n12;
								path_mpu[n12]=n12;
							end
						else if((~a&~b&c)==1)
							begin
								node_mpu=1;
								en_mpu=n13;
								
								path_mpu[n13]=n14;
								path_mpu[n14]=n14;
							end
						else if((~a&b&~c)==1)
							begin
								node_mpu=1;
								en_mpu=n12;
								
								path_mpu[n13]=n13;
								path_mpu[n12]=n13;
							end
						else if((~a&b&c)==1)
							begin
								node_mpu=2;
								en_mpu=n12;
								
								path_mpu[n13]=n14;
								path_mpu[n12]=n13;
								path_mpu[n14]=n14;
							end
						else if((a&~b&c)==1)
							begin
								node_mpu=2;
								en_mpu=n13;
								
								path_mpu[n13]=n14;
								path_mpu[n14]=n12;
								path_mpu[n12]=n12;
							end
							
						nst=calculation_ssu;
				end
					
 calculation_ssu:
				begin
						if(adj[7][5]==64)
							a=0;
						else a=1;
						if(adj[5][6]==64)
							b=0;
						else b=1;
						if(adj[6][7]==64)
							c=0;
						else c=1;
							
						if((a&b&c)==1)
							begin
								node_ssu=3;
								en_ssu=n8;
								
								path_ssu[n8]=n6;
								path_ssu[n6]=n7;
								path_ssu[n7]=n8;
							end
						else if((a&b&~c)==1)
							begin
								node_ssu=2;
								en_ssu=n8;
								
								path_ssu[n8]=n6;
								path_ssu[n6]=n7;
								path_ssu[n7]=n7;
							end
						else if((a&~b&~c)==1)
							begin
								node_ssu=1;
								en_ssu=n8;
								
								path_ssu[n8]=n6;
								path_ssu[n6]=n6;
							end
						else if((~a&~b&c)==1)
							begin
								node_ssu=1;
								en_ssu=n7;
								
								path_ssu[n7]=n8;
								path_ssu[n8]=n8;
							end
						else if((~a&b&~c)==1)
							begin
								node_ssu=1;
								en_ssu=n6;
								
								path_ssu[n6]=n7;
								path_ssu[n7]=n7;
							end
						else if((~a&b&c)==1)
							begin
								node_ssu=2;
								en_ssu=n6;
								
								path_ssu[n6]=n7;
								path_ssu[n7]=n8;
								path_ssu[n8]=n8;
							end
						else if((a&~b&c)==1)
							begin
								node_ssu=2;
								en_ssu=n7;
								
								path_ssu[n7]=n8;
								path_ssu[n8]=n6;
								path_ssu[n6]=n6;
							end
							
						nst=calculation_pu;
					end
	
	calculation_pu:	begin
							if(adj[2][3]==64)	
								a=0;
							else a=1;
							if(adj[2][4]==64)	
								b=0;	
							else b=1;
							
								c=0;
									
							if((a&b)==1)
							begin
								node_pu=3;
								en_pu=n3;
						
								
								path_pu[n3]=n4;
								path_pu[n4]=n5;
								path_pu[n5]=n3;
								
							end
							else if((~a&b)==1)
							begin
								node_pu=1;
								en_pu=n5;
								
								
								path_pu[n5]=n3;
								path_pu[n3]=n3;
							end				
							else if((a&~b)==1)
							begin
								node_pu=1;
								en_pu=n3;
							
								
								path_pu[n3]=n4;
								path_pu[n4]=n4;
							end
									
						nst=calculation_w;	
						end
						
	calculation_w:	begin
						
					if(adj[8][10]==64)	
								a=0;
							else a=1;
							if(adj[8][9]==64)	
								b=0;	
							else b=1;
							
								c=0;
									
							if((a&b)==1)
							begin
								node_w=2;
								en_w=n9;
								
								path_w[n9]=n11;
								path_w[n11]=n10;
								path_w[n10]=n9;
							end
							else if((~a&b)==1)
							begin
								node_w=1;
								en_w=n10;
	
								path_w[n10]=n9;
								path_w[n9]=n9;
							end				
							else if((a&~b)==1)
							begin
								node_w=1;
								en_w=n9;
			
								
								path_w[n9]=n11;
								path_w[n11]=n11;
							end
							
						nst=idle;   // go to dijkstra		
						
						
						if(adj[n2][n14]==64)                           // code to initializise source node for the first time
							s_node=en_pu;
						else s_node=en_mpu;	
							
						end
///////////////////////////end of initialization///////////////////////

//states for traversal [measure---cleft---cright---fleft---fright---uturn---luturn]///////////////////////////
	measure:	begin 
				
				wait_count=0;
				
				is_stop=0;
				 prev =measure;
				 nst=measure;
				
				
				if(center>left)		//center_left = absolute(center-left)
					center_left=center-left;
				else
					center_left=left-center;
				
				if(center>right)		//center_right = absolute(center-right)
					center_right=center-right;
				else
					center_right=right-center;
		
				if(center<600)			//making decision whether to actuate right/left motor
				begin
				if(center_left>center_right)
					nst=cright;			//correct the path and move towards left (right motor actuated)
					
				if(center_left<center_right)
					nst=cleft;			//correct the path and move towards right (left motor actuated) 
				end
				
				else						//if no change in direction then go straight 
				begin
					nst=measure;
					dirl=1;
					dirr=1;
					pwml=base_pwm;
					pwmr=base_pwm;
				end
				
				// node detection 
				if ((center>600) && (left > 600) && (right > 600))		//condition for node
				begin
					if (node==0)
						begin
						
							node_detect=1;							//1 if node is detected else 0
							node=1;									//making sure each node is detected only once
							
							if (curn==s_node)
								nst=scan_init;
								
							else
								nst=decision;                       // else next action to be executed will be decided in delay state 

						end
				end
			
				else
					begin	
						node			=0;
						node_detect	=0;								//node not detected
					end
					
			
	//		// bridge code
			if((prevn==n13)&&(curn==n7))
			begin
				 ddelay=11750; 
				if ((accel_delay<1200000)) // a small delay so that bot reaches the bridge before execution of code which consideres accelerometer values
				begin
					accel_delay=accel_delay+1;
					dirl=1;
					dirr=1;
					base_pwm=100;
					offset1 =0;		//high speed for the bridge ascending
					offset2 =100;
				end
						
						
				else 
				begin
					if ((accel < 65530) && (accel > 65300))  // acceleration values while bot moving up the slope 
					begin
						dirl=1;
						dirr=1;
						base_pwm=100;
						offset1 =0;		//high speed for the bridge ascending
						offset2 =100;
						slope_count=0;
					end
					
//					else 
//					begin
//						dirl=1;
//						dirr=1;
//						base_pwm=35;
//						offset1 =36;		//high speed for the bridge ascending
//						offset2 =35;
//						slope_count=0;
//					end
						
								
					else if ((accel > 65530) && (accel <30)) // acceleration values while bot is on the bridge
					begin
						dirr=1;
						dirl=1;
						slope_count=0;
						base_pwm = 35;	//base speed (pwm) of motors 
						offset1 = 36;		//base will be increased by offest1 
						offset2= 35;
					end
								
					else if ((accel >30) && (accel < 200 )&& (slope_count<1200000)) // acceleration values while bot moving down the slope 
					begin
						slope_count=slope_count+1;   
						dirl=0;
						dirr=0;
						base_pwm	=0; 
						offset1 	=0;		//base will be increased by offest1 30
						offset2	=30;
					end
								
					else if (slope_count>1200000) // this condition prevents the bot from getting stopped while descending down the slope.
					begin
						dirl=1;
						dirr=1;
						base_pwm=35;
						offset1=36;
						offset2=35;
					end
			end 
					
		end 		//	[ if ((accel_delay<1200000)&& (node_count==8)) ....else begin ]'s end
		
		else
		begin
			base_pwm = 43;	//base speed (pwm) of motors 
			offset1 = 37;		//base will be increased by offest1 
			offset2= 43;
			 //ddelay=490000; 
		end
		
		
//		if((prevn==n14)&&(curn==n2))
//		begin
//			base_pwm = 39;	//base speed (pwm) of motors 
//			offset1 = 35;		//base will be increased by offest1 
//			offset2= 39;
//		end

		
							
	end
				
	cleft:	begin			
										//actuating left motor	
				if(center_right-center_left>range)			//condition for setting left motor pwm
				begin

					if ((dirl==1) && (dirr==1))				
					begin
					pwml=(base_pwm+offset1);					// correcting the path in forward direction 
					pwmr=(base_pwm-offset2);
					end
					else begin 
					pwml=(base_pwm-offset1);					// correcting the path in reverse direction
					pwmr=(base_pwm+offset2);
					end					
					
				end
				
				else begin											//else for going straight  							
					
					pwml=base_pwm;
					pwmr=base_pwm;
					dirl=1'b1;										//forward direction of motors
					dirr=1'b1;
					
				end
				
				if (prev==measure)
				nst=measure;	
				else if (prev==scan_traversal)
				nst=scan_traversal;
				
				end
				
	cright:	begin													//actuating right motor
				if(center_left-center_right>range)			//condition for setting right motor pwm
				begin
					if((dirl==1) && (dirr==1))
						begin
						pwml=(base_pwm-offset2);					// correcting the path in forward direction
						pwmr=(base_pwm+offset1);
						end
					else
						begin
						pwml=(base_pwm+offset2);					// correcting the path in reverse direction
						pwmr=(base_pwm-offset1);
						end
				end
				
				else
				begin
					pwml=base_pwm;									//else for going straight
					pwmr=base_pwm;
				
					dirl=1'b1;										//forward direction of motors
					dirr=1'b1;		
				end
				
				if (prev==measure)
					nst=measure;	
				else if (prev==scan_traversal)
					nst=scan_traversal;
								
				end
				
	fleft:	begin					// turn left
				dirl=1'b0;
				dirr=1'b1;
				pwml=turn_pwn_r;
				pwmr=turn_pwn;
				
				if((center>600) && (wait_count>500000)) // a small delay to introduce a forcefull turn so that bot's center line sensor moves away from the current path to white patch.  																
					begin
					
					if (prev==measure)
						nst=measure;	
					else if (prev==scan_traversal)
						nst=scan_traversal;
						
					wait_count=0;
					
					end
					
				else
				begin
					nst=fleft;
					wait_count=wait_count+1;
				end
				
				end
				
	fright:	begin				// turn right
				dirl=1'b1;
				dirr=1'b0;
				pwml=turn_pwn;
				pwmr=turn_pwn_r;
			
				if((center>600) && (wait_count>500000)) // a small delay to introduce a forcefull turn so that bot's center line sensor moves away from the current path to white patch.
					begin
					
					if (prev==measure)
						nst=measure;	
					else if (prev==scan_traversal)
						nst=scan_traversal;
						
					wait_count=0;
					end
					
				else
				begin
					nst=fright;
					wait_count=wait_count+1;
				end
				
				end	
			
  uturn :begin
				dirl=1'b1;
				dirr=1'b0;
				pwml=turn_pwn;
				pwmr=turn_pwn_r;
			
				if((center>600) && (wait_count>3000000)) // a small delay to introduce a forcefull turn so that bot's center line sensor moves away from the current path to white patch.
					begin
					if (prev==measure)
						nst=measure;	
					else if (prev==scan_traversal)
						nst=scan_traversal;
					wait_count=0;
					end
					
				else
				begin
					nst=uturn;
					wait_count=wait_count+1;
				end	
			end
			
  luturn :begin
				dirl=1'b0;
				dirr=1'b1;
				pwml=turn_pwn_r;
				pwmr=turn_pwn;
			
				if((center>600) && (wait_count>3000000)) // a small delay to introduce a forcefull turn so that bot's center line sensor moves away from the current path to white patch.
					begin
					if (prev==measure)
						nst=measure;	
					else if (prev==scan_traversal)
						nst=scan_traversal;
					wait_count=0;
					end
					
				else
				begin
					nst=luturn;
					wait_count=wait_count+1;
				end	
			end
			
///////////////////////////start of path algorithms////////////////////////////////////////////////
	
	idle:	begin 
			pos=6'b111111;
			dijk_count=0;
			temp=255;
			for(i=0;i<14;i=i+1)
			begin
				if (i != s_node)
					dist[i] = 64; // initialising distance matrix with source having distance 0 and others infinity
				else
					dist[i] = 0;
				visited[i] = 0;	 // none of the points visited
			end
	
			for (i=0;i<14;i=i+1)
			begin
				if (i != s_node)
					path[i] = -1; // a previous array, which stores the optimal point from which the current point should be reached
				else
					path[i] = s_node; // previous of source is source itself
			end
			
			//dijk_nst=s1;
			nst=s1;
			
			
			
			
			end
	
			
	s1:	begin
			/////////////////calculates minimum/////////////////
			for (i=0;i<14;i=i+1)
				begin
					if ((temp > dist[i] ) && (visited[i] != 2'b11)) // finds a point adjacent to current point and is closer than otheers
					begin
						temp = dist[i];
						pos = i;
					end
				end
			
			visited[pos]=2'b11;
			temp=255;
			nst=s2;//dijkstra;
			///////////////////////////////////////////////////
			end
	
	s2:	begin				//inplements dijkstra
			for (k=0;k<14;k=k+1)
			begin
				if ((visited[k] == 0 ) && (adj[pos][k] != 0) ) // if not visited and is adjacent to the current point
				begin
					 if (dist[k] > (adj[pos][k] + dist[pos]) ) // if new path with lesser cost is found
					 begin
						  dist[k] = (adj[pos][k] + dist[pos]); //update the path and cost
						  path[k] = pos;
					 end
				end
			end///for end
			nst=s3;//dijkstra;
			end
			
	s3:	begin
			dijk_count=dijk_count+1;
			if(dijk_count<14)
			begin
				nst=s1;//dijkstra;
			end
			else
			begin
				nxtn=path[curn];
				prev=measure;
				nst=decision;//decision
				
			end	
			
			end	

	// scan initialization state , here once the traversal from one unit to another is complete, the path array
									// is re-initialzed with the path of the corresponding unit, along with the source node 
									// for the next unit , and the no. of nodes the bot needs to detect while scanning are also initialized. 					
							
	scan_init:	begin
					if(curn==en_mpu)
					begin
						unit=1;
						node_count=node_mpu;
						for(x=0;x<14;x=x+1)
						begin
							if(path_mpu[x]!=-1)
								path[x]=path_mpu[x];
						end
						s_node=en_ssu;
					end
					
					else if(curn==en_pu)
					begin
						unit=2;
						node_count=node_pu;
						for(x=0;x<14;x=x+1)
						begin
							if(path_pu[x]!=-1)
								path[x]=path_pu[x];
						end
						s_node=en_ssu;
					end
						
					else if(curn==en_w)
					begin
						unit=3;
						node_count=node_w;
						for(x=0;x<14;x=x+1)
						begin
							if(path_w[x]!=-1)
								path[x]=path_w[x];
						end
						s_node=en_ssu;
					end
						
					else if(curn==en_ssu)
					begin
						adj[6][12]=64;
						adj[12][6]=64;
						unit=4;
						
						//red patch condition
						
					
						
						
						for(x=0;x<14;x=x+1)
						begin
							if(path_ssu[x]!=-1)
								path[x]=path_ssu[x];
						end
						
						
						if(mpu_count>0)
							s_node=en_mpu;
						else if(pu_count>0)
							s_node=en_pu;
						else if(w_count>0)
							s_node=en_w;
							
						if(red_flagg==1)
							s_node=en_ssu;
						else
							s_node=s_node;
						
					end
						

					nxtn	=	path[curn]; // to uodate the nxtn for the new updated path matrix. 	nst=decision;
					prev	=	scan_traversal;
					nst	=	decision;
					
					end
						
scan_traversal:	begin
				is_stop=0;
				in_scan=1;
				
				if((unit==1))
				begin
					base_pwm = 38;	//base speed (pwm) of motors 
					offset1 = 42;		//base will be increased by offest1 
					offset2= 38;		//base will be decreased by offest2
				end
				else if((unit==2)||(unit==3))
				begin
					base_pwm = 41;	//base speed (pwm) of motors 
					offset1 = 43;		//base will be increased by offest1 
					offset2= 41;		//base will be decreased by offest2
				end
				
				else if((unit==4))
				begin
					base_pwm = 36;	//base speed (pwm) of motors 
					offset1 = 42;		//base will be increased by offest1 
					offset2= 36;		//base will be decreased by offest2
				end
				
				
				
				prev =scan_traversal;
				
				if(center>left)		//center_left = absolute(center-left)
					center_left=center-left;
				else
					center_left=left-center;
				
				if(center>right)		//center_right = absolute(center-right)
					center_right=center-right;
				else
					center_right=right-center;
				
			
		
				if(center<600)			//making decision whether to actuate right/left motor
				begin
				if(center_left>center_right)
					nst=cright;			//correct the path and move towards left (right motor actuated)
					
				if(center_left<center_right)
					nst=cleft;			//correct the path and move towards right (left motor actuated) 
				end
				
				else						//if no change in direction then go straight 
				begin
					
					nst=scan_traversal;
					dirl=1;
					dirr=1;
					pwml=base_pwm;
					pwmr=base_pwm;
				
				end
				
				
				
				
				
				// node detection 
				if ((center>600) && (left > 600) && (right > 600))		//condition for node
				begin
					if (node==0)
						begin
						
							node_detect=1;							//1 if node is detected else 0
							node=1;									//making sure each node is detected only once
							
							
							if((node_count>1)&&(node_count<=3))
							begin
								prevn=curn;
								curn=nxtn;
								nxtn=path[curn];
							end
							
							node_count=node_count-1;
							
						end
				end
				
				else
				begin
					if(node_count==0)
					begin
						nst=idle;
						in_scan=0;
						if((unit==4)&&(mpu_count==0)&&(pu_count==0)&&(w_count==0))	// condition to stop the bot when all the faults are detected
						begin
							is_stop=1;
							nst=run_end;
						end
						else
							nst=idle;
						unit=0;
						
					end
					node=0;
					node_detect=0;								//node not detected
				end
				
				if((prevn==n4)&&(curn==n5)&&(nxtn==n3))		// special condition to incorporate virtual nodes we have introduced
				begin
					prevn=n5;
					curn=n3;
					nxtn=n4;
					node_count=node_count-1;
				end
					
					

					
				end
						
				 
	decision :  begin					// this state dictates what course of action need to be taken on every node. 
				
				case(curn)
				
				
				n1:	begin
						prevn=n1;
						curn=n2;
						nst=measure;
						nxtn=path[curn];
						end
						
				n2:	begin
						nxtn=path[curn];                // special case just for node 2
						
						if((prevn==n1) && (nxtn==n3))
							nst=fright;
							
						else if((prevn==n1) && (nxtn==n14))
							nst=fleft;
							
						else if((prevn==n3) && (nxtn==n14))
							nst=measure;
							
						else if((prevn==n14) && (nxtn==n3))
							nst=measure;
							
						else if(prevn==nxtn)                 // will only be executed in case of scan orientation 
							nst=uturn;
							
						prevn=n2;
						curn=nxtn;
						nxtn=path[curn];
						
						end
						
				n3:	begin
						if((prevn==n2) && (nxtn==n4))
							nst=fleft;
							
						else if((prevn==n2) && (nxtn==n5))
							nst=fright;
							
						else if((prevn==n4) && (nxtn==n2))
							nst=fright;
							
						else if((prevn==n5) && (nxtn==n2))
							nst=fleft;
							
						else if((prevn==n5) && (nxtn==n4))
							nst=measure;
							
						else if((prevn==n4) && (nxtn==n5))
							nst=measure;
							
						else if((prevn==n5) && (nxtn==n5))                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						else if((prevn==n4) && (nxtn==n4))                 // will only be executed in case of scan orientation 
							nst=uturn;
							
						prevn=n3;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n4:	begin
						if((prevn==n6) && (nxtn==n3))
							nst=fright;
							
						else if((prevn==n3) && (nxtn==n6))
							nst=fleft;
						
						else if(prevn==nxtn)                 
							nst=luturn;
						
					
						
						prevn=n4;
						curn=nxtn;
						nxtn=path[curn];
						end
						
				n5:	begin
						if((prevn==n6) && (nxtn==n3))
							nst=fleft;
							
						else if((prevn==n3) && (nxtn==n6))
							nst=fright;
							
						else if(prevn==nxtn)                 // will only be executed in case of scan orientation 
							nst=uturn;
						
						prevn=n5;
						curn=nxtn;
						nxtn=path[curn];
						end
					
					
				n6:	begin
						if(((prevn==n4)||(prevn==n5)) && (nxtn==n7))
							nst=fleft;
							
						else if(((prevn==n4)||(prevn==n5)) && (nxtn==n8))
							nst=fright;
							
						else if((prevn==n8) && (nxtn==n7))
							nst=measure;
							
						else if((prevn==n7) && (nxtn==n8))
							nst=measure;
							
						else if((prevn==n8) && ((nxtn==n4)||(nxtn==n5)))
							nst=fleft;
						
						else if((prevn==n7) && ((nxtn==n4)||(nxtn==n5)))
							nst=fright;
							
						else if((prevn==n8) && (nxtn==n8))                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						else if((prevn==n7) && (nxtn==n7))                 // will only be executed in case of scan orientation 
							nst=uturn;
						
						prevn=n6;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n7:	begin
						if((prevn==n13) && (nxtn==n8))
							nst=fleft;
							
						else if((prevn==n13) && (nxtn==n6))
							nst=fright;
							
						else if((prevn==n6) && (nxtn==n8))
							nst=measure;
							
						else if((prevn==n6) && (nxtn==n13))
							nst=fleft;
							
						else if((prevn==n8) && (nxtn==n6))
							nst=measure;
							
						else if((prevn==n8) && (nxtn==n13))
							nst=fright;
							
						else if((prevn==n6) && (nxtn==n6))                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						else if((prevn==n8) && (nxtn==n8))                 // will only be executed in case of scan orientation 
							nst=uturn;
							
						prevn=n7;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n8:	begin
						if((prevn==n6) && (nxtn==n7))
							nst=measure;
							
						else if((prevn==n6) && (nxtn==n9))
							nst=fright;
							
						else if((prevn==n7) && (nxtn==n9))
							nst=fleft;
							
						else if((prevn==n7) && (nxtn==n6))
							nst=measure;
							
						else if((prevn==n9) && (nxtn==n6))
							nst=fleft;
							
						else if((prevn==n9) && (nxtn==n7))
							nst=fright;
							
						else if((prevn==n7) && (nxtn==n7))                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						else if((prevn==n6) && (nxtn==n6))                 // will only be executed in case of scan orientation 
							nst=uturn;
							
						else if((prevn==n8) && (nxtn==n8) && (curn==n8))
							nst=measure;
							
						prevn=n8;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n9:	begin
						if((prevn==n11) && (nxtn==n10))
							nst=measure;
							
						else if((prevn==n11) && (nxtn==n8))
							nst=fright;
							
						else if((prevn==n10) && (nxtn==n11))
							nst=measure;
							
						else if((prevn==n10) && (nxtn==n8))
							nst=fleft;
							
						else if((prevn==n8) && (nxtn==n11))
							nst=fleft;
							
						else if((prevn==n8) && (nxtn==n10))
							nst=fright;
							
						else if((prevn==n11) && (nxtn==n11))                 // will only be executed in case of scan orientation 
							nst=uturn;
						
						else if((prevn==n10) && (nxtn==n10))                 // will only be executed in case of scan orientation 
							nst=luturn;
							
						prevn=n9;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n10:	begin
						if((prevn==n9) && (nxtn==n12))
							nst=fright;
							
						else if((prevn==n12) && (nxtn==n9))
							nst=fleft;
							
						else if(prevn==nxtn)                 // will only be executed in case of scan orientation 
							nst=uturn;
						
						prevn=n10;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n11:	begin
						if((prevn==n12) && (nxtn==n9))
							nst=fright;
							
						else if((prevn==n9) && (nxtn==n12))
							nst=fleft;
							
						else if(prevn==nxtn)                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						prevn=n11;
						curn=nxtn;
						nxtn=path[curn];
						end
						
						
				n12:	begin
						if(((prevn==n10)||(prevn==n11)) && (nxtn==n13))
							nst=fleft;
							
						else if(((prevn==n10)||(prevn==n11)) && (nxtn==n14))
							nst=fright;
							
						else if((prevn==n14) && (nxtn==n13))
							nst=measure;
							
						else if((prevn==n13) && (nxtn==n14))
							nst=measure;
							
						else if((prevn==n14) && ((nxtn==n10)||(nxtn==n11)))
							nst=fleft;
						
						else if((prevn==n13) && ((nxtn==n10)||(nxtn==n11)))
							nst=fright;
							
						else if((prevn==n14) && (nxtn==n14))                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						else if((prevn==n13) && (nxtn==n13))                 // will only be executed in case of scan orientation 
							nst=uturn;
						
						prevn=n12;
						curn=nxtn;
						nxtn=path[curn];
						end
						
				n13:	begin
						if((prevn==n12) && (nxtn==n7))
							nst=fleft;
							
						else if((prevn==n14) && (nxtn==n7))
							nst=fright;
							
						else if((prevn==n14) && (nxtn==n12))
							nst=measure;
							
						else if((prevn==n7) && (nxtn==n14))
							nst=fleft;
							
						else if((prevn==n12) && (nxtn==n14))
							nst=measure;
							
						else if((prevn==n7) && (nxtn==n12))
							nst=fright;
							
						else if((prevn==n12) && (nxtn==n12))                 // will only be executed in case of scan orientation 
							nst=luturn;
						
						else if((prevn==n14) && (nxtn==n14))                 // will only be executed in case of scan orientation 
							nst=uturn;
							
						prevn=n13;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				n14:	begin
				
						if((prevn==n12) && (nxtn==n13))
							nst=measure;
							
						else if((prevn==n12) && (nxtn==n2))
							nst=fright;
							
						else if((prevn==n13) && (nxtn==n2))
							nst=fleft;
							
						else if((prevn==n13) && (nxtn==n12))
							nst=measure;
							
						else if((prevn==n2) && (nxtn==n12))
							nst=fleft;
							
						else if((prevn==n2) && (nxtn==n13))
							nst=fright;
							
						else if((prevn==n12) && (nxtn==n12))                 // will only be executed in case of scan orientation 
							nst=uturn;
						
						else if((prevn==n13) && (nxtn==n13))                 // will only be executed in case of scan orientation 
							nst=luturn;
							
						prevn=n14;
						curn=nxtn;
						nxtn=path[curn];
						end
				
				endcase
					
				if((nst==measure)&&(prev==scan_traversal))
					nst=scan_traversal;
				
				temp_state=nst;
				nst=turn_delay;
				
				
			end	 //[decision: begin]'s end 
	
	turn_delay:	begin
				
					if((temp_state==fleft)||(temp_state==fright)||(temp_state==uturn)||(temp_state==luturn))
					begin
						if (wait_count<ddelay)	// a small delay so that before executing any condition the bot crosses the node. 	  
						begin
							wait_count=wait_count+1;
							nst=turn_delay;
						end
					
						else 
						begin
							wait_count=0;
							nst=temp_state;
						end
					end
					
					else	nst=temp_state;
					
					end
					
	run_end:		begin						// stops the bot
					is_stop=1;
					pwml=0;
					pwmr=0;
					nst=run_end;
	
					end
	
	endcase		 //[case(nst)]'s endcase
	
	
end 				//[always..begin]'s end


endmodule 