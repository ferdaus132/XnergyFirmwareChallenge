//put your definition here
//definition of constants
#define PICurrentKp	0.5   //proportional constant for PI current control loop
#define PICurrentKi	0.1   //Integral constant for PI current control loop
#define PICurrentMax 	1.0  //Upper Limit for output and integral for PI current control loop
#define PICurrentMin 	-1.0 //Lower Limit for output and integral for PI current control loop
#define CurrentScale 	0.01  //Scaling factor for current feedback and reference to value 1.0
				//Scaling factor is 1/absolute_max_current_reading
				//Example for absolute max 100A, current_scale is 0.01

#define PIVoltageKp	0.5   //proportional constant for PI voltage control loop
#define PIVoltageKi	0.1   //Integral constant for PI voltage control loop
#define PIVoltageMax 	1.0  //Upper Limit for output and integral for PI voltage control loop
#define PIVoltageMin 	-1.0 //Lower Limit for output and integral for PI voltage control loop
#define VoltageScale 	0.01 //Scaling factor for voltage feedback and reference to value 1.0
			  	//Scaling factor is 1/absolute_max_voltage_reading
			  	//Example for absolute max 100V, voltage_scale is 0.01
#define StateDL 0 //state idle
#define StateCC 1 //state constant current
#define StateCV 2 //state constant voltage

#define NetworkInitialization 0
#define NetworkPreoperational 1
#define NetworkOperational 2

#define _TRUE  1 //Value to emulate true in integer
#define _FALSE 0 //value to emulate valse in integer

//CAN struct example
typedef struct {
	uint8_t Data[8];
	uint16_t Length;
	uint32_t ID;
} CAN_msg_typedef;

//variable declaration for CAN
CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

//function prototyping for CAN
void CAN_write(CAN_msg_typedef *msg);
bool CAN_read(CAN_msg_typedef *msg); //return true if there is received msg

//struct for PI variables
typedef struct {
    float  fErr; 	// Data: control error value or proportional term
    float  fIntg; 	// Data: integral term  
    float  fKp; 	// Parameter: proportional loop gain          
    float  fKi; 	// Parameter: integral gain
    float  fKc; 	// Parameter: sensor scaling
    float  fReff; 	// Input: reference set-point
    float  fFb; 	// Input: feedback
    float  fMaxOut;// Parameter: upper saturation limit
    float  fMinOut;// Parameter: lower saturation limit
    float  fOut; 	// Output: controller output 
} fPIVar;

//Default Value for PI variables
#define DefaultPIVar {   \
			0.0, \
			0.0, \
			0.0, \
			0.0, \
			0.0, \
			0.0, \
			0.0, \
			0.0, \
			0.0, \
			0.0  \
}

//procedure to calculate PI
void CalcPI(fPIVar *PIVar){
	//calculation of error control between reference and feedback
	PIVar->fErr = PIVar->fReff-PIVar->fFb;

	//calculation of integral 
	PIVar->fIntg = PIVar->fIntg + ((PIVar->fErr)*PIVar->fKi);

	//desaturation for integral
	if(PIVar->fIntg > PIVar->fMaxOut){PIVar->fIntg=PIVar->fMaxOut;}
	if(PIVar->fIntg < PIVar->fMinOut){PIVar->fIntg=PIVar->fMinOut;}

	//PI output calculation
	PIVar->fOut = (PIVar->fErr * PIVar->fKp) + PIVar->fIntg;

	//limit of PI output
	if(PIVar->fOut > PIVar->fMaxOut){PIVar->fOut=PIVar->fMaxOut;}
	if(PIVar->fOut < PIVar->fMinOut){PIVar->fOut=PIVar->fMinOut;}

}
/****************************/
//declaration of Variables for PI control loops
/****************************/
fPIVar PICurrent = DefaultPIVar;
fPIVar PIVoltage = DefaultPIVar;

/******************************/
//declaration of global variables
//some variable names defined in the challenge questions
/******************************/
uint8_t MainState = StateDL;
uint8_t NetworkState = NetworkInitialization;
uint8_t enable_command = _FALSE;
uint16_t time_rx=0;
uint16_t time_tx=0;
uint32_t time_ms=0;
float Iref;
float Vref;
float Imin;
float current_feedback;
float current_reference;
float voltage_feedback;
float voltage_reference;
float voltage_hysband;
float minimum_current;

//procedure for variable initialization
void Initialization(void){
//initialize your variables here
	PICurrent.fKi=PICurrentKi;
	PICurrent.fKp=PICurrentKp;
	PICurrent.fKc=CurrentScale;
	PICurrent.fMaxOut=PICurrentMax;
	PICurrent.fMinOut=PICurrentMin;
	PICurrent.fIntg=0;

	PIVoltage.fKi=PIVoltageKi;
	PIVoltage.fKp=PIVoltageKp;
	PIVoltage.fKc=VoltageScale;
	PIVoltage.fMaxOut=PIVoltageMax;
	PIVoltage.fMinOut=PIVoltageMin;
	PIVoltage.fIntg=0;

	minimum_current = Imin; //Imin is user defined or defined from BMS through CAN
	voltage_reference = Vref; //Vref is user defined or defined from BMS through CAN
	current_reference = Iref; //Iref is user defined or defined from BMS through CAN
	voltage_hysband = 0.1*voltage_reference; //voltage hysteresis band to debounce state transition
	MainState = StateDL; //default state is idle
} 

//routine for control algorithm
void control_routine(void){
//run the control algorithm here

	if(MainState!=StateDL){
		if(MainState==StateCV){
			/************************/
			//PI voltage control loop
			/************************/
			//voltage_reference -> from user defined ref from BMS trhough CAN
			//voltage_feedback -> from voltage value from voltage sensor/ADC
			PIVoltage.fFb   = voltage_feedback*PIVoltage.fKc;
			PIVoltage.fReff = voltage_reference*PIVoltage.fKc;
			CalcPI(&PIVoltage);
			//output of voltage control loop is PIVoltage.fOut
			//PIVoltage.fOut is used as current_reference 
			//and limited by Iref value for constant current mode
		}

		/*************************/
		//PI current control loop
		/*************************/
		//current_reference = Iref for constant current, 
		//current_reference = PIVoltage.fOut for constant voltage
		//current_feedback -> from voltage value from voltage sensor/ADC
		PICurrent.fFb = current_feedback*PICurrent.fKc;
		if(MainState==StateCC){
			PICurrent.fReff = current_reference*PICurrent.fKc;
		}else{
			PICurrent.fReff = PIVoltage.fOut;
		}
		CalcPI(&PICurrent);
		//use PICurrent.fOut as final control variable to modulate 
	}else{
		PICurrent.fIntg=0;
		PICurrent.fOut=0;
	}
	
	/**********************/
	//place output modulation code here
	//final control value is stored in PICurrent.fOut, range value from -1.0 to 1.0
	//modulate final control value to the modulator 
	//by scale it accordingly and depends on the modulator used
	
	time_ms++; //assume INT frequency is 1kHz, for timing purpose
	time_rx++; //ticker 1ms for can rx
	time_tx++; //ticker 1ms for can tx

} 

//procedure of main state machine
void main_state_machine(void){
//run the state transition here

	/**************************/
	//main state machine
	/**************************/
	switch(MainState){
		case StateDL: 
				if(enable_command == _TRUE){ //wait for enable command
					MainState=StateCC;	//set state to constant current
				}
				break;
		case StateCC:	//wait for the voltage reach voltage reference
				if(voltage_feedback >= voltage_reference){
					//set state to constant voltage							
					MainState=StateCV; 
					//initiate integral value for voltage control loop 
					//for smooth transitioning
					PIVoltage.fIntg=PICurrent.fReff;
				}
				break;
		case StateCV: 	//if voltage drop below reference and hysteresis band, 
				//change state to constant current 
				if(voltage_feedback < (voltage_reference-voltage_hysband)){ 
					//set state to constant current if voltage drop below 
					//reference and hysteresis band
					MainState=StateCC; 
				}else{
					//if current decreased below minimum current, set state to idle
					if(current_feedback < minimum_current){ 
						MainState=StateDL;	//set state to idle`
						enable_command = _FALSE; //set enable_command to false
					}
				}
				break;
		default: 
				MainState=StateDL; //set default state to idle
				break;
	}
	
	if(enable_command == _FALSE){ 
		MainState=StateDL; //set state to idle if enable = False
	}
}

void CAN_write_handler(void){
//CAN tx
	float float_temp;
	//set write ID to 0x181
	Can_tx.ID      = 0x181;
	//set length data to 5 (5 byte)
	Can_tx.Length  = 5;
	//convert voltage_feedback*10 to uint16_t, and store MSB to data[0]
	Can_tx.Data[0] = ((uint16_t)(voltage_feedback*10.0)>>8);
	//convert voltage_feedback*10 to uint16_t, and store LSB to data[1]
	float_temp = voltage_feedback*10.0;
	Can_tx.Data[1] = (uint16_t)float_temp;
	//convert current_feedback*10 to uint16_t, and store MSB to data[2]
	Can_tx.Data[2] = ((uint16_t)(current_feedback*10.0)>>8);
	//convert current_feedback*10 to uint16_t, and store LSB to data[2]
	float_temp = current_feedback*10.0;
	Can_tx.Data[3] = (uint16_t)float_temp;
	if(MainState == StateDL){
		Can_tx.Data[4] = 0; //if main state is idle, send status not charging
	}else{
		Can_tx.Data[4] = 1; //if not idle, send status as charging
	}

}

void CAN_read_handler(void){
//CAN rx
	//local temp variable declaration
	uint16_t can_read_temp;
	//read voltage reference from data[0] (MSB) and data[1] (LSB)
	can_read_temp = Can_rx.Data[0];
	can_read_temp = (can_read_temp<<8)|((uint16_t)Can_rx.Data[1]&0x00FF);
	voltage_reference = (float)can_read_temp*0.1;
	//read current reference from data[2] (MSB) and data[3] (LSB)
	can_read_temp = Can_rx.Data[2];
	can_read_temp = (can_read_temp<<8)|((uint16_t)Can_rx.Data[3]&0x00FF);
	current_reference = (float)can_read_temp*0.1;
	//read enable command from data[4];
	enable_command = Can_rx.Data[4];
	
	if(enable_command == _TRUE){
		//set network state to operational if enable command true
		NetworkState = NetworkOperational;
	}else{
		//set network state to preoperational if enable command false
		NetworkState = NetworkPreoperational;
	}
}

void network_management(void){
//run the network management here
	if(NetworkState!=NetworkInitialization){
		//check and read incoming message from BMS
		if(CAN_read(&Can_rx)){
			//handle data from can reading if message received
			CAN_read_handler();
			//reset rx timer
			time_rx=0;
		}else{
			if(time_rx>=5000){ //handler whine CAN timeout occured (5s)
				//set network state to operational
				NetworkState = NetworkPreoperational;
				//stop charging command
				enable_command = _FALSE;
				//set main state to idle
				MainState = StateDL;
				//reset rx timer
				time_rx=0;
			}
		}

		//in network state operational, send data to BMS every 200ms
		if(NetworkState=NetworkOperational){
			if(time_tx>=200){ //handler when transmit timer reach 200ms
				CAN_write_handler(); //run data write handler
				CAN_write(&Can_tx); // write data to CAN bus
				time_tx=0; //reset transmit timer
			}
		}
	}
	
	//send heart beat data every 1s
	if(time_ms>=1000){
		//set CAN ID for heartbeat
		Can_tx.ID=0x701;
		//set data length for heartbeat
		Can_tx.Length=1;
		//set data[0] from networkstate
		Can_tx.Data[0]=NetworkState;
		//write data to CAN bus
		CAN_write(&Can_tx);
		//reset heartbeat timer
		time_ms=0;
	}
} 


void main(void){
	Initialization();
	PieVectTable.EPWM1_INT = &control_routine;
	while(true){
		main_state_machine();
		network_management();
	}
}
