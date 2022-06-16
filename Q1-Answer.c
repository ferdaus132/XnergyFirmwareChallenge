//put your definition here
//definition of constants
#define PICurrentKp	0.5   //proportional constant for PI current control loop
#define PICurrentKi	0.1   //Integral constant for PI current control loop
#define PICurrentMax 	1.0  //Upper Limit for output and integral for PI current control loop
#define PICurrentMin 	-1.0 //Lower Limit for output and integral for PI current control loop
#define CurrentScale 	0.01 //Scaling factor for current feedback and reference to value 1.0
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

#define _TRUE  1 //Value to emulate true in integer
#define _FALSE 0 //value to emulate valse in integer

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
uint8_t enable_command = _FALSE;
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

void main(void){
	Initialization();
	PieVectTable.EPWM1_INT = &control_routine;
	while(true){
		main_state_machine();
	}
}
