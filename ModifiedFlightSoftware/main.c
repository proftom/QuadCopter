#include "thal.h"
#include "mavlink.h"
#include <math.h>

// *** Config stuff
#define FIRMWARE_VERSION    1           // Firmware version
#define MESSAGE_LOOP_HZ     15          // Max frequency of messages in Hz (keep this number low, like around 15)
#define RX_PANIC            2           // Number of seconds after missing RX before craft considered "disconnected"

// *** LED stuff
unsigned char flashPLED, flashVLED, flashRLED;

float gim_ptemp;

// *** ILink stuff
ilink_identify_t ilink_identify;
ilink_thalstat_t ilink_thalstat;
ilink_thalctrl_t ilink_thalctrl;
ilink_imu_t ilink_rawimu;
ilink_imu_t ilink_scaledimu;
ilink_altitude_t ilink_altitude;
ilink_attitude_t ilink_attitude;
ilink_attitude_t ilink_attitude_rx;
ilink_thalparam_t ilink_thalparam_tx;
ilink_thalparam_t ilink_thalparam_rx;
ilink_thalpareq_t ilink_thalpareq;
ilink_iochan_t ilink_inputs0;
ilink_iochan_t ilink_outputs0;
ilink_position_t ilink_position;

void LinkInit(void);

// *** Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned short RxWatchdog;
unsigned short UltraWatchdog;

// *** Functions
void SensorZero(void);
void CalibrateGyro(void);
void CalibrateGyroTemp(unsigned int seconds);
void CalibrateMagneto(void);

// *** Configs
#define FAST_RATE           400
#define SLOW_RATE           75
/*
#define INPUT_RATE          50
#define ALT_RATE            25*/

#define ZEROTHROTMAX        1*FAST_RATE

#define SLOW_DIVIDER        FAST_RATE/SLOW_RATE
/*#define INPUT_DIVIDER       AHRS_RATE/INPUT_RATE
#define ALT_DIVIDER         AHRS_RATE/ALT_RATE*/

unsigned short slowSoftscale;//, inputSoftscale, baroSoftscale;

// TODO: check ESC response at THROTTLEOFFSET, consider raising THROTTLEOFFSET to 1000
#define THROTTLEOFFSET    900		// Corresponds to zero output PWM. Nominally 1000=1ms, but 800 works better
#define IDLETHROTTLE    	175		// Minimum PWM output to ESC when in-flight to avoid motors turning off
#define MAXTHROTTLE     	1200	// Maximum PWM output to ESC (PWM = THROTTLEOFFSET + MAXTHROTTLE)
#define MAXTHROTTLEPERCENT	0.9     // Maximum percentage throttle should be (reserves some extra output for stabilisation at high throttle).

#define OFFSTICK        	50
#define MIDSTICK        	512		// Corresponds to input when stick is in middle (approx value).
#define MAXSTICK        	850		// Corresponds to input when stick is at the top

#define MAXTHRESH           (MAXSTICK+MIDSTICK)/2 - 50
#define MINTHRESH           (MIDSTICK+OFFSTICK)/2 + 50
//#define CONSTRAIN       600   //Constraint on maximum motor demands


// *** Parameters
typedef struct paramStorage_struct {
    char name[16];
    float value;
} paramStorage_t;

struct paramStorage_struct paramStorage[] = {
    {"DRIFT_AKp",           0.2f},
    {"DRIFT_MKp",      0.02f},   
    #define DRIFT_AccelKp   paramStorage[0].value    
    #define DRIFT_MagKp     paramStorage[1].value  
  
    {"SPR_ULTRA",       0.95f},  
    #define SPR_ULTRA 		paramStorage[2].value

    {"YAW_SEN",     0.00002f},     
    {"PITCH_SEN",    0.0022f},    
    {"ROLL_SEN",     0.0022f},   
    #define YAW_SENS        paramStorage[3].value
    #define PITCH_SENS      paramStorage[4].value
    #define ROLL_SENS       paramStorage[5].value

    {"YAW_DZN",      0.001f},
    #define YAW_DEADZONE    paramStorage[6].value 

    {"PITCH_Kp",      400.0f},     
    {"PITCH_Ki",        2.0f},    
    {"PITCH_Kd",      100.0f},     
    {"PITCH_Kdd",    1500.0f},
    {"PITCH_Bst",     0.0f},      
    {"PITCH_De",      0.999f},    
    #define PITCH_Kp        paramStorage[7].value 
    #define PITCH_Ki        paramStorage[8].value
    #define PITCH_Kd        paramStorage[9].value 
    #define PITCH_Kdd       paramStorage[10].value 
    #define PITCH_Boost     paramStorage[11].value 
    #define PITCH_De        paramStorage[12].value 

    {"ROLL_Kp",       400.0f},       
    {"ROLL_Ki",         2.0f},      
    {"ROLL_Kd",       100.0f},        
    {"ROLL_Kdd",     1500.0f},     		
    {"ROLL_Bst",       0.00f},    				
    {"ROLL_De",       0.999f},
    #define ROLL_Kp         paramStorage[13].value  
    #define ROLL_Ki         paramStorage[14].value 
    #define ROLL_Kd         paramStorage[15].value 
    #define ROLL_Kdd        paramStorage[16].value 	
    #define ROLL_Boost      paramStorage[17].value 
    #define ROLL_De         paramStorage[18].value

    {"YAW_Kp",        1000.0f},     
    {"YAW_Kd",        250.0f},  
    {"YAW_Bst",        0.00f},   
    #define YAW_Kp          paramStorage[19].value 
    #define YAW_Kd          paramStorage[20].value 
    #define YAW_Boost       paramStorage[21].value 	

    // Mode
    {"MODE_SIMP",       0.0f},  // Simplicity Mode  (0 for off 1 for on)
    {"MODE_ULTRA",      1.0f},  // Ultrasound Mode
    {"MODE_GIM",        1.0f},	// Gimbal Lock
    {"MODE_SWTCH",      5.0f}, 	// AUX1 SWITCH: 1 to toggle simplicity, 2 to toggle Ultra, 3 to toggle Gimbal, 4 to toggle GPS

    #define MODE_SIMP 		paramStorage[22].value
    #define MODE_ULTRA 		paramStorage[23].value
    #define MODE_GIM 		paramStorage[24].value
    #define MODE_SWTCH 		paramStorage[25].value
    //Limits
    {"LIM_ANGLE",       0.4f},  // Roll and Pitch Angle Limit in Radians
    {"LIM_ALT",         1000.0f},  // Altitude Limit in metres when in Ultrasound Mode
    #define LIM_ANGLE 		paramStorage[26].value
    #define LIM_ALT 		paramStorage[27].value

    // Magneto Correction
    {"CAL_MAGN1",     0.001756f},      
    {"CAL_MAGN2",   0.00008370f},        
    {"CAL_MAGN3",   0.00005155f},     	
    {"CAL_MAGN5",     0.001964f},    		
    {"CAL_MAGN6",   0.00002218f},     
    {"CAL_MAGN9",     0.001768f},     
    {"CAL_MAGM1",       0.0f},     
    {"CAL_MAGM2",        0.0f},     	
    {"CAL_MAGM3",        0.0f},  

    #define MAGCOR_N1       paramStorage[28].value 
    #define MAGCOR_N2       paramStorage[29].value 		
    #define MAGCOR_N3       paramStorage[30].value 		
    #define MAGCOR_N5       paramStorage[31].value 
    #define MAGCOR_N6       paramStorage[32].value 
    #define MAGCOR_N9       paramStorage[33].value 
    #define MAGCOR_M1       paramStorage[34].value 		
    #define MAGCOR_M2       paramStorage[35].value 
    #define MAGCOR_M3       paramStorage[36].value

    // Ultrasound
    //{"ULTRA_Kp",        0.05f},
    //{"ULTRA_Kd",        5.0f},
    //{"ULTRA_Ki",        0.00001f},
    {"ULTRA_Kp",        0.03f},
    {"ULTRA_Kd",        3.0f},
    {"ULTRA_Ki",        0.00001f},
    {"ULTRA_De",      	0.9999f},
    {"ULTRA_TKOFF",   	250.0f}, 
    {"ULTRA_LND",   	70.0f}, 
    #define ULTRA_Kp        paramStorage[37].value 
    #define ULTRA_Kd        paramStorage[38].value 
    #define ULTRA_Ki        paramStorage[39].value 
    #define ULTRA_De        paramStorage[40].value
    #define ULTRA_TKOFF     paramStorage[41].value 
    #define ULTRA_LND       paramStorage[42].value 

    {"CAL_GYROX",   0.0f},
    {"CAL_GYROY",   0.0f},
    {"CAL_GYROZ",   0.0f},
    #define CAL_GYROX       paramStorage[43].value 
    #define CAL_GYROY       paramStorage[44].value 
    #define CAL_GYROZ       paramStorage[45].value 
    
	{"DETUNE",			0.2f},
	#define DETUNE		paramStorage[46].value
	
	{"LIM_RATE",			100.0f},
	#define LIM_RATE		paramStorage[47].value
    
	{"LIM_ULTRA",			0.35f},
	#define LIM_ULTRA		paramStorage[48].value
    
	{"ULTRA_DRMP",     3.0f}, 
	{"ULTRA_DTCT",     10.0f},
    #define ULTRA_DRMP      paramStorage[49].value
    #define ULTRA_DTCT      paramStorage[50].value
	
	{"LIM_THROT", 		0.3f},
	#define LIM_THROT		paramStorage[51].value
	
	{"ULTRA_OVDEC",		0.05f},
	#define ULTRA_OVDEC		paramStorage[52].value
	
	{"ULTRA_DEAD",		100},
	#define ULTRA_DEAD		paramStorage[53].value
	
	{"ULTRA_OVTH",		10},
	#define ULTRA_OVTH		paramStorage[54].value
	
	{"MODE_GPS",      0.0f}, 	
    #define MODE_GPS 		paramStorage[55].value
	
	{"GPS_Kp",      0.05f}, 	
	{"GPS_Kde",      1.0f}, 	
	{"GPS_Ki",     	0.0f}, 	
	{"GPS_Kd",     	0.1f}, 	
    #define GPS_Kp		paramStorage[56].value
    #define GPS_Kde		paramStorage[57].value
    #define GPS_Ki		paramStorage[58].value
    #define GPS_Kd		paramStorage[59].value
    
    {"GPA_ALTKp", 10.0f},
    {"GPA_ALTKi", 0.1f},
    {"GPA_ALTKde", 0.999f},
    {"GPA_ALTKd", 0.0f},
    #define GPS_ALTKp		paramStorage[60].value
    #define GPS_ALTKi		paramStorage[61].value
    #define GPS_ALTKde		paramStorage[62].value
    #define GPS_ALTKd		paramStorage[63].value
   
    {"GPS_ALTHo", 2.5f},
    #define GPS_ALTHo       paramStorage[64].value
    
    {"CAL_AUTO", 1.0f},
    #define CAL_AUTO        paramStorage[65].value    
    
    
     //Gimbal in the Pitch Axis	
    {"GIM_PMID",     1500.0f},		//Mid Point 
    {"GIM_PSCAL",    -1650.0f},		//Scaling
    {"GIM_PLIMH",    2300.0f}, 		//Limit High
    {"GIM_PLIML",     700.0f},		//Limit Low
    {"GIM_PPLUS",     100.0f},		//Limit Low
    #define GIM_PMID        paramStorage[66].value 		
    #define GIM_PSCAL       paramStorage[67].value 
    #define GIM_PLIMH       paramStorage[68].value 		
    #define GIM_PLIML       paramStorage[69].value 
    #define GIM_PPLUS       paramStorage[70].value 

    //Gimbal in the Roll Axis
    {"GIM_RMID",     1500.0f},       //Mid Point    	
    {"GIM_RSCAL",    1650.0f},      //Scaling
    {"GIM_RLIMH",    2300.0f},     	//Limit High
    {"GIM_RLIML",     700.0f},		//Limit Low
    #define GIM_RMID        paramStorage[71].value 
    #define GIM_RSCAL       paramStorage[72].value
    #define GIM_RLIMH       paramStorage[73].value 
    #define GIM_RLIML       paramStorage[74].value
	
	{"SPR_OUT",       0.3f},  
    #define SPR_OUT 		paramStorage[75].value
    
    {"MODE_AUTO",       0.0f},
    #define MODE_AUTO       paramStorage[76].value

    /*{"CURVE_X1",       30.0f},    
    {"CURVE_Y1",       45.0f},  
    {"CURVE_X2",       55.0f},    
    {"CURVE_Y2",       70.0f},    
    {"CURVE_Yn",       70.0f}, 
    #define THROTTLE_X1     paramStorage[8].value 
    #define THROTTLE_Y1     paramStorage[9].value  
    #define THROTTLE_X2     paramStorage[10].value 
    #define THROTTLE_Y2     paramStorage[11].value
    #define THROTTLE_Yn     paramStorage[12].value   */
    /*{"ADJ_UP",   1.0f},      // Ultrasound minimimumum throttle
    {"ADJ_DOWN",   30.0f},      // Ultrasound minimimumum throttle
    #define ADJ_UP          paramStorage[73].value 
    #define ADJ_DOWN        paramStorage[74].value */
	};

unsigned int paramSendCount;
unsigned int paramCount;
unsigned char paramSendSingle;

void EEPROMLoadAll(void);
void EEPROMSaveAll(void);

#define EEPROM_MAX_PARAMS   100 // this should be greater than or equal to the above number of parameters
#define EEPROM_OFFSET   0 // EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess)
#define EEPROM_VERSION	18 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

// *** quaternion storage
float q1, q2, q3, q4;
float thetaAngle, phiAngle, psiAngle;
float MM1, MM2, MM3, MM4, MM5, MM6, MM7, MM8, MM9;

float lat_diff;
float lon_diff;
float lat_diff_i;
float lon_diff_i;
float lat_diff_d;
float lon_diff_d;
float lon_diff_old;
float lat_diff_old;
float alt_diff_old;
float alt_diff;
float alt_diff_i;
float alt_diff_d;
float alt_throttle;
float gpsThrottle;


typedef struct{
	float demandav;
    float demand;
	float demandtemp;
	float demandtempold;
	float derivativetemp;
	float secondderivativetemp;
	float derivativetempold;
	float demandOld;
	float valueOld;
	float derivative;
	float integral;
} directionStruct;

directionStruct pitch;
directionStruct roll;
directionStruct yaw;

typedef struct {
    float value;
    float valueOld;
    float error;
    float demand;
    float demandav;
    float derivative;
    float integral;
	float demandincr;
	float demandold;
} altStruct;
altStruct alt;

float thetaAngle, phiAngle, psiAngle, psiAngleinit;

// *** Sensor Storage
#define GAV_LEN 6
#define AAV_LEN 80
#define MAV_LEN 6

typedef struct{
    volatile signed short raw;
    volatile float av;
    volatile float value;
    volatile float offset;
    volatile float error;
    volatile float verterror;
    volatile signed int total;
    signed short history[GAV_LEN];
} sensorStructGyro;

typedef struct{
    sensorStructGyro X;
    sensorStructGyro Y;
    sensorStructGyro Z;
    unsigned int count;
} threeAxisSensorStructGyro;

typedef struct{
    volatile signed short raw;
    volatile float av;
    volatile float value;
    volatile signed int total;
    signed short history[AAV_LEN];
} sensorStructAccel;

typedef struct{
    sensorStructAccel X;
    sensorStructAccel Y;
    sensorStructAccel Z;
    unsigned int count;
} threeAxisSensorStructAccel;

typedef struct{
    volatile signed short raw;
    volatile float av;
    volatile float value;
    volatile signed int total;
    signed short history[AAV_LEN];
} sensorStructMag;

typedef struct{
    sensorStructMag X;
    sensorStructMag Y;
    sensorStructMag Z;
    unsigned int count;
} threeAxisSensorStructMag;

threeAxisSensorStructGyro Gyro;
threeAxisSensorStructAccel Accel;
threeAxisSensorStructMag Mag;

void ReadGyroSensors(void);
void ReadAccelSensors(void);
void ReadMagSensors(void);

// *** Input stuff
unsigned short rcInput[7];
unsigned int rxLoss;
unsigned int rxFirst;
signed short yawtrim;
signed short throttletrim;
float throttle;
float nonLinearThrottle(float input);
unsigned int auxState, flapState;

float pitchcorrectionav, rollcorrectionav, yawcorrectionav;

// *** Ultrasound
float ultra;
float ultraav;
unsigned int ultraLoss;
float ultrathrottle;
float ultraTkOffThrottle;
unsigned int ultraTkOffInput;

unsigned int airborne;
unsigned int landing;
unsigned int throttleHoldOff;
// *** Output stuff
float motorN, motorE, motorS, motorW;
float motorNav, motorEav, motorSav, motorWav;
float tempN;
float tempE;
float tempS;
float tempW;

float pitch_dd;
float roll_dd;

float throttleold;

float CFDC_mag_x_zero_local;
float CFDC_mag_y_zero_local;
float CFDC_mag_z_zero_local;
float CFDC_mag_x_zero_local_derotate;
float CFDC_mag_y_zero_local_derotate;
float CFDC_mag_z_zero_local_derotate;
float CFDC_mag_x_zero_local_unelipse;
float CFDC_mag_y_zero_local_unelipse;
float CFDC_mag_z_zero_local_unelipse;
float CFDC_mag_x_zero_local_unelipse_decentre;
float CFDC_mag_y_zero_local_unelipse_decentre;
float CFDC_mag_z_zero_local_unelipse_decentre;
float CFDC_mag_x_zero;
float CFDC_mag_y_zero;
float CFDC_mag_z_zero;
float CFDC_mag_x_error;
float CFDC_mag_y_error;
float CFDC_mag_z_error;

float CFDC_qdiff1;
float CFDC_qdiff2;
float CFDC_qdiff3;
float CFDC_qdiff4;
float CFDC_iq1_zero;
float CFDC_iq2_zero;
float CFDC_iq3_zero;
float CFDC_iq4_zero;

float CFDC_count;

unsigned int yaw_lock;

// *** ARM thingy
unsigned int armed, calib, zeroThrotCounter;
void Arm(void);
void Disarm(void);


// *** Button stuff
unsigned int PRGBlankTimer; // Blanking time for button pushes
unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

// ****************************************************************************
// *** Initialiseation
// ****************************************************************************

// *** Initialiser function: sets everything up.
void setup() {
    // *** LED setup
        LEDInit(PLED | VLED);
        LEDOff(PLED | VLED);
        
        flashPLED = 0;
        flashVLED = 0;
        flashRLED = 0;
    
        armed = 0;
        calib = 0;
        zeroThrotCounter = 0;
    
    // *** Timers and couters6
        rxLoss = 50;
        ultraLoss = ULTRA_OVTH + 1;
        sysMS = 0;
        sysUS = 0;
        SysTickInit();  // SysTick enable (default 1ms)

    // *** Parameters
        paramCount = sizeof(paramStorage)/20;
        EEPROMLoadAll();
        paramSendCount = paramCount;
        paramSendSingle = 0;
    
    // *** Establish ILink
        ilink_thalstat.sensorStatus = 1; // set ilink status to boot
        ilink_thalstat.flightMode = (0x1 << 0); // attitude control
        ilink_identify.deviceID = WHO_AM_I;
        ilink_identify.firmVersion = FIRMWARE_VERSION;
        ILinkInit(SLAVE);
    
    // *** Initialise input
		throttle = 0;
		rxFirst = 0;
        auxState = 0;
        RXInit();
		
        
    // *** Initialise Ultrasound
        UltraInit();
        UltraFast();
		alt.demandincr =  0;
        
    // *** Battery sensor
        ADCInit(CHN7);
        ADCTrigger(CHN7);
		Delay(1);
        ilink_thalstat.battVoltage = (ADCGet() * 6325) >> 10; // Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
    
    // *** Calibrate Sensors
        Delay(500);
        SensorInit();
        SensorZero();
		
	// Current Field Distortion Compensation initialisation
		CFDC_count = 0;

    // *** Ultrasound
        UltraInit();
        UltraFast();
    
    // *** quaternion AHRS init
        q1 = 1;
        q2 = 0;
        q3 = 0;
        q4 = 0;
		
    // *** Rotation matrix Initialisation
        MM1 = 1;
        MM2 = 0;
        MM3 = 0;
        MM4 = 0;
		MM5 = 1;
        MM6 = 0;
        MM7 = 0;
		MM8 = 0;
        MM9 = 1;
		
		pitch_dd = 0;
		roll_dd = 0;
        
        /*thetaAngle = 0;
        phiAngle = 0;
        psiAngle = 0;
        
        pitchcorrectionav = 0;
        rollcorrectionav = 0;
        yawcorrectionav = 0;*/
        
    // *** Timer for AHRS
        // Set high confidence in accelerometer/magneto to rotate AHRS to initial heading
        float tempAccelKp = DRIFT_AccelKp;
        float tempMagKp = DRIFT_MagKp;
        DRIFT_MagKp *= 100;
        DRIFT_AccelKp *= 100;
        
        //float tempQAMK = QAMK;
        //QAMK = 1;
        
        Timer0Init(59);
        Timer0Match0(1200000/FAST_RATE, INTERRUPT | RESET);
        Delay(1000);
        
        //QAMK = tempQAMK;
	
        DRIFT_MagKp = tempMagKp;
        DRIFT_AccelKp = tempAccelKp;
    
        slowSoftscale = 0;
        //inputSoftscale = 3;
        //baroSoftscale = 7;
    
	// *** Initialise PWM outputs
        motorN = 0;
        motorE = 0;
        motorS = 0;
        motorW = 0;
		
		motorNav = 0;
        motorEav = 0;
        motorSav = 0;
        motorWav = 0;
		
		throttleold = 0;
        
    // *** Initialise timers and loops
        //PWMInit(PWM_NESW);
		PWMInit(PWM_X | PWM_Y);
		gim_ptemp = 0;
        RITInitms(1000/MESSAGE_LOOP_HZ);
        flashPLED = 0;
        LEDOff(PLED);
		
		lat_diff = 0;
		lon_diff = 0;
		lat_diff_i = 0;
		lon_diff_i = 0;
		lat_diff_d = 0;
		lon_diff_d = 0;
		
		
		yaw_lock = 0;
}

void Arm(void) {

    if(CAL_AUTO > 0) {
        CalibrateGyroTemp(1);
    }

    PWMInit(PWM_NESW);
    PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
    if(armed == 0) {
        Delay(500);
        PWMSetNESW(THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE);
        Delay(100);
        PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
        Delay(300);
        PWMSetNESW(THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE);
        Delay(100);
        PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
    }
    armed = 1;
    
    psiAngleinit = psiAngle; // simplicity reference
    ilink_thalstat.sensorStatus &= ~(0x7); // mask status
    ilink_thalstat.sensorStatus |= 4; // active/armed
}

void Disarm(void) {
    if(armed) {
        PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
        Delay(100);
        
        PWMSetN(THROTTLEOFFSET + IDLETHROTTLE);
        Delay(100);
        PWMSetN(THROTTLEOFFSET);
        Delay(33);
        PWMSetE(THROTTLEOFFSET + IDLETHROTTLE);
        Delay(100);
        PWMSetE(THROTTLEOFFSET);
        Delay(33);
        PWMSetS(THROTTLEOFFSET + IDLETHROTTLE);
        Delay(100);
        PWMSetS(THROTTLEOFFSET);
        Delay(33);
        PWMSetW(THROTTLEOFFSET + IDLETHROTTLE);
        Delay(100);
        PWMSetW(THROTTLEOFFSET);

        Delay(100);
    }
    PWMSetNESW(0, 0, 0, 0);
    armed = 0;
    ilink_thalstat.sensorStatus &= ~(0x7); // mask status
    ilink_thalstat.sensorStatus |= 3; // standby
}

void CalibrateMagneto(void) {
        unsigned int i;
        unsigned int  good;
        float Xav, Yav, Zav;
        float distance;
        
        signed short Xmax, Xmin, Ymax, Ymin, Zmax, Zmin;
        
        unsigned int started;
    
        if(armed == 0) {
            ilink_thalstat.sensorStatus = 2; // calibrating
        
            started = 0;
            
            Xav = 0;
            Yav = 0;
            Zav = 0;

            flashVLED = 1;
            good = 0;
            
            ReadMagSensors();
            Xmax = Mag.X.raw;
            Xmin = Xmax;
            Ymax = Mag.Y.raw;
            Ymin = Ymax;
            Zmax = Mag.Z.raw;
            Zmin = Zmax;
            
            // Wait for Gyro to be steady or 20 seconds
            for(i=0; i<5000; i++) {
                ReadGyroSensors();
                
                // calculate distance of data from running average
                distance  = (Xav - (float)Gyro.X.raw)*(Xav - (float)Gyro.X.raw);
                distance += (Yav - (float)Gyro.Y.raw)*(Yav - (float)Gyro.Y.raw);
                distance += (Zav - (float)Gyro.Z.raw)*(Zav - (float)Gyro.Z.raw);
                
                if(started == 0) {
                    // before starting, wait for gyro to move around
                    if(distance > 2000) {
                        // high-movement, increment good counter and add average value.
                        good++;
                        if(good >= 10) {
                            started = 1; // if enough movement readings, escape loop
                            good = 0;
                            flashVLED = 2;
                        }
                    }
                    else {
                        good = 0;
                    }
                }
                else {
                    ReadMagSensors();
                    if(Mag.X.raw > Xmax) Xmax = Mag.X.raw;
                    else if(Mag.X.raw < Xmin) Xmin = Mag.X.raw;
                    if(Mag.Y.raw > Ymax) Ymax = Mag.Y.raw;
                    else if(Mag.Y.raw < Ymin) Ymin = Mag.Y.raw;
                    if(Mag.Z.raw > Zmax) Zmax = Mag.Z.raw;
                    else if(Mag.Z.raw < Zmin) Zmin = Mag.Z.raw;
                    
                    if(distance < 2000) {
                        // high-movement, increment good counter and add average value.
                        good++;
                        if(good >= 200) break; // if enough movement readings, escape loop
                    }
                    else {
                        good = 0;
                    }
                }
                
                Xav *= 0.95f;
                Xav += 0.05f * (float)Gyro.X.raw;
                Yav *= 0.95f;
                Yav += 0.05f * (float)Gyro.Y.raw;
                Zav *= 0.95f;
                Zav += 0.05f * (float)Gyro.Z.raw; 
                
                Delay(14);
            }
            
            MAGCOR_M1 = (Xmax + Xmin)/2;
            MAGCOR_M2 = (Ymax + Ymin)/2;
            MAGCOR_M3 = (Zmax + Zmin)/2;
            EEPROMSaveAll();
            
            flashPLED = 0;
            LEDOff(PLED);
        
            ilink_thalstat.sensorStatus &= ~(0x7); // mask status
            ilink_thalstat.sensorStatus |= 3; // standby
    }
}

// *** Calibrates all the sensors
void SensorZero(void) {
    unsigned int i;
    signed short data[4];
    
    if(!GetGyro(data) || !GetMagneto(data) || !GetAccel(data) || GetBaro() == 0) {
        LEDInit(PLED | VLED);
        LEDOn(PLED);
        LEDOff(VLED);
        flashPLED = 2;
        flashVLED = 2;
        while(1);
    }
    
    // *** Zero totals
        Gyro.X.total = 0;
        Gyro.Y.total = 0;
        Gyro.Z.total = 0;
        Accel.X.total = 0;
        Accel.Y.total = 0;
        Accel.Z.total = 0;
        Mag.X.total = 0;
        Mag.Y.total = 0;
        Mag.Z.total = 0;
        
        Gyro.X.error = 0;
        Gyro.Y.error = 0;
        Gyro.Z.error = 0;

        Gyro.X.verterror = 0;
        Gyro.Y.verterror = 0;
        Gyro.Z.verterror = 0;

        for(i=0; (i<GAV_LEN); i++) {
            Gyro.X.history[i] = 0;
            Gyro.Y.history[i] = 0;
            Gyro.Z.history[i] = 0;
        }
        
        for(i=0; (i<AAV_LEN); i++) {
            Accel.X.history[i] = 0;
            Accel.Y.history[i] = 0;
            Accel.Z.history[i] = 0;
        }
        
        for(i=0; (i<MAV_LEN); i++) {
            Mag.X.history[i] = 0;
            Mag.Y.history[i] = 0;
            Mag.Z.history[i] = 0;
        }
        
        Gyro.X.offset = 0;
        Gyro.Y.offset = 0;
        Gyro.Z.offset = 0;
        
        // pre-seed averages
        for(i=0; (i<GAV_LEN); i++) {
            ReadGyroSensors();
        }
        
        for(i=0; (i<AAV_LEN); i++) {
            ReadAccelSensors();
        }
        
        for(i=0; (i<MAV_LEN); i++) {
            ReadMagSensors();
        }
	
        ilink_thalstat.sensorStatus |= (0xf << 3);
}

void CalibrateGyro(void) {
    CalibrateGyroTemp(6);
    CAL_GYROX = Gyro.X.offset;
    CAL_GYROY = Gyro.Y.offset;
    CAL_GYROZ = Gyro.Z.offset;
    EEPROMSaveAll();
}

void CalibrateGyroTemp(unsigned int seconds) {
    unsigned int i;
    // *** Calibrate Gyro
        unsigned int  good;
        float Xav, Yav, Zav;
        signed int Xtotal, Ytotal, Ztotal;
        float distance;
        
        if(armed == 0) {
            
            ReadGyroSensors();
            Xtotal = 0;
            Ytotal = 0;
            Ztotal = 0;
            Xav = Gyro.X.raw;
            Yav = Gyro.Y.raw;
            Zav = Gyro.Z.raw;

            flashPLED = 1;
            good = 0;
            
            // Wait for Gyro to be steady or 20 seconds
            for(i=0; i<5000; i++) {
                ReadGyroSensors();
                
                // calculate distance of data from running average
                distance  = (Xav - (float)Gyro.X.raw)*(Xav - (float)Gyro.X.raw);
                distance += (Yav - (float)Gyro.Y.raw)*(Yav - (float)Gyro.Y.raw);
                distance += (Zav - (float)Gyro.Z.raw)*(Zav - (float)Gyro.Z.raw);
                
                if(distance < 2000) {
                    // low-movement, increment good counter and add average value.
                    good++;
                    Xtotal += Gyro.X.raw;
                    Ytotal += Gyro.Y.raw;
                    Ztotal += Gyro.Z.raw;
                    if(good >= 333) break; // if enough good readings, escape loop
                }
                else {
                    // high movement, zero good counter, and average values.
                    good = 0;
                    
                    Xtotal = 0;
                    Ytotal = 0;
                    Ztotal = 0;
                }
                
                Xav *= 0.95f;
                Xav += 0.05f * (float)Gyro.X.raw;
                Yav *= 0.95f;
                Yav += 0.05f * (float)Gyro.Y.raw;
                Zav *= 0.95f;
                Zav += 0.05f * (float)Gyro.Z.raw; 
                
                Delay(3);
            }
            
            ilink_thalstat.sensorStatus &= ~(0x7); // mask status
            ilink_thalstat.sensorStatus |= 2; // calibrating

            flashPLED=2;
            LEDOn(PLED);
            
            // at this point should have at least 200 good Gyro readings, take some more
                
            for(i=0; i<seconds*333; i++) {
                ReadGyroSensors();
                
                Xtotal += Gyro.X.raw;
                Ytotal += Gyro.Y.raw;
                Ztotal += Gyro.Z.raw;
                
                Delay(3);
            }

            Gyro.X.offset = (float)Xtotal/(float)((seconds+1) * 333);
            Gyro.Y.offset = (float)Ytotal/(float)((seconds+1) * 333);
            Gyro.Z.offset = (float)Ztotal/(float)((seconds+1) * 333);
            
            /*CAL_GYROX = Gyro.X.offset;
            CAL_GYROY = Gyro.Y.offset;
            CAL_GYROZ = Gyro.Z.offset;
            EEPROMSaveAll();*/
            
            flashPLED = 0;

            LEDOff(PLED);
        
            ilink_thalstat.sensorStatus &= ~(0x7); // mask status
            ilink_thalstat.sensorStatus |= 3; // standby
        }
}
// ****************************************************************************
// *** Main loop and timer loops
// ****************************************************************************

// *** Main loop, nothing much happens in here.
void loop() {
    //if(idleCount < IDLE_MAX) idleCount++; // this is the counter for CPU idle time
    // *** Deal with button push
        if(PRGBlankTimer == 0) {
            if(PRGTimer > 3000) {
                RXBind();
                PRGPushTime = 0;
                PRGTimer = 0;
                PRGBlankTimer = 200;
            }
        }
    
    __WFI();
}

// *** SysTick timer: deals with general timing
void SysTickInterrupt(void) {
    sysMS += 1;
    sysUS += 1000;
    
    // *** deal with flashing LEDs
        if(sysMS % 25 == 0) {
            if(sysMS % 100 == 0) {
                if(flashPLED) LEDToggle(PLED);
                if(flashVLED) LEDToggle(VLED);
                if(flashRLED) LEDToggle(RLED);
            }
            else {
                if(flashPLED == 2) LEDToggle(PLED);
                if(flashVLED == 2) LEDToggle(VLED);
                if(flashRLED == 2) LEDToggle(RLED);
            }
        }
    
    // *** time the button pushes
        if(PRGPoll() == 0) PRGTimer++;
        else {
            PRGPushTime = PRGTimer;
            PRGTimer = 0;
            if(PRGBlankTimer) PRGBlankTimer--;
        }
}

// *** RIT interrupt, deal with timed iLink messages.
void RITInterrupt(void) {
    
    // *** deal with iLink parameter transmission
        unsigned int i;
        if(paramSendCount < paramCount) {
            unsigned short thisParam = paramSendCount; // store this to avoid race hazard since paramSendCount can change outside this interrupt
            ilink_thalparam_tx.paramID = thisParam;
            ilink_thalparam_tx.paramValue = paramStorage[thisParam].value;
            ilink_thalparam_tx.paramCount = paramCount;
            for(i=0; i<16; i++) {
                ilink_thalparam_tx.paramName[i] = paramStorage[thisParam].name[i];
                if(paramStorage[thisParam].name[i] == '\0') break;
            }
            if(ILinkSendMessage(ID_ILINK_THALPARAM, (unsigned short *) & ilink_thalparam_tx, sizeof(ilink_thalparam_tx)/2 -1)) {
                if(paramSendSingle) {
                    paramSendSingle = 0;
                    paramSendCount = paramCount;
                }
                else {
                    paramSendCount = thisParam+1;
                }
            }
        }
}


// ****************************************************************************
// *** Loops and shit
// ****************************************************************************

void Timer0Interrupt0() {
    if(++slowSoftscale >= SLOW_DIVIDER) {
        slowSoftscale = 0;
        
        // *** Read Magneto
        ReadMagSensors();
        
        
        // *** RC Input
            if(RXGetData(rcInput)) {
                if(rxLoss > 10) rxLoss -= 10;
                ilink_inputs0.channel[0] = rcInput[RX_THRO];
                ilink_inputs0.channel[1] = rcInput[RX_AILE];
                ilink_inputs0.channel[2] = rcInput[RX_ELEV];
                ilink_inputs0.channel[3] = rcInput[RX_RUDD];
                ilink_inputs0.channel[4] = rcInput[RX_AUX1];
                ilink_inputs0.channel[5] = rcInput[RX_FLAP];
                ilink_inputs0.isNew = 1;
				
				if(rxFirst < 25) {
					yawtrim = rcInput[RX_RUDD];
					throttletrim = rcInput[RX_THRO];
                    if(rcInput[RX_AUX1] > MIDSTICK) {
                        // Controller's Aux switch on startup
                        MODE_SIMP = 0;
                    }
                    else {
                        MODE_SIMP = 1;
                    }
					rxFirst++;
				}
                
                // Controller's aux or gear switch
                if(rcInput[RX_AUX1] > MIDSTICK) {
                    if(auxState == 1 || auxState == 0) {
                        // do something on switch
                        if (MODE_SWTCH == 1) {
                            MODE_SIMP = 0;
                            ilink_thalstat.flightMode &= ~(0x1 << 2);
                        }
                        if (MODE_SWTCH == 2) {
                            MODE_ULTRA = 0;
                            ilink_thalstat.flightMode &= ~(0x1 << 3);
                        }
                        if (MODE_SWTCH == 3) MODE_GIM = 0;
                        if (MODE_SWTCH == 4) {
                            MODE_GPS = 0;
                            ilink_thalstat.flightMode &= ~(0x1 << 4);
                        }
                        if (MODE_SWTCH == 5) {
                            MODE_AUTO = 0;
                        }
                    }
                    auxState = 2;
                }
                else {
                    if(auxState == 2 || auxState == 0) {
                        // do something on switch
                        if (MODE_SWTCH == 1) {
                            MODE_SIMP = 1;
                            ilink_thalstat.flightMode |= (0x1 << 2);
                        }
                        if (MODE_SWTCH == 2) {
                            MODE_ULTRA = 1;
                            ilink_thalstat.flightMode |= (0x1 << 3);
                        }
                        if (MODE_SWTCH == 3) MODE_GIM = 1;
                        if (MODE_SWTCH == 4) {
                            MODE_GPS = 1;
                            ilink_thalstat.flightMode |= (0x1 << 4);
                        }
                        if (MODE_SWTCH == 5) {
                            MODE_AUTO = 1;
                        }
                    }
                    auxState = 1;
                }
                
                // Controller's flap switch
                if(rcInput[RX_FLAP] < MIDSTICK) {
                    if(flapState == 1) {
                        // do something on switch
                    }
                    flapState = 0;
                }
                else {
                    if(flapState == 0) {
                        // do something on switch
						gim_ptemp += 1;
						if(gim_ptemp > 6) gim_ptemp = -6;
                    }
                    flapState = 1;
                }
                
                // *** TODO: Futaba S.Bus support
                // *** TODO: state setting
                // *** TODO: non-lineariser
                
                flashVLED = 0;
                LEDOff(VLED);
            }
            else {
                rxLoss ++;
                if(rxLoss > 50) {
                    rxLoss = 50;
                    // RC signal lost
                    // *** TODO: Perform RC signal loss state setting
                    if(armed) PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
                    ilink_outputs0.channel[0] = THROTTLEOFFSET;
                    ilink_outputs0.channel[1] = THROTTLEOFFSET;
                    ilink_outputs0.channel[2] = THROTTLEOFFSET;
                    ilink_outputs0.channel[3] = THROTTLEOFFSET;
                    flashVLED = 2;
                }
            }
            
        // *** Read Ultrasound
            ultra = (UltraGetNewRawData()) * 0.17;
            
			//ilink_attitude.roll = ultra; //temporarily acquired
			
            if(ultra > 0)  {
					
                ultraLoss = 0;
				
				if(ultra - ultraav > LIM_ULTRA/SLOW_DIVIDER) ultra = ultraav + LIM_ULTRA/SLOW_DIVIDER;
				if(ultraav - ultra > LIM_ULTRA/SLOW_DIVIDER) ultra = ultraav - LIM_ULTRA/SLOW_DIVIDER;
                
                ultraav *= SPR_ULTRA;
                ultraav += (1-SPR_ULTRA) * ultra;
                
                alt.valueOld = alt.value;
                alt.value = ultraav;

                ilink_altitude.relAlt = ultraav;
				
			}
            else {
                ultraLoss++;
				if(ultraLoss > ULTRA_OVTH) ultraLoss = ULTRA_OVTH;
            }
			
			
			
			
        // *** Read battery voltage
            ilink_thalstat.battVoltage = (ADCGet() * 6325) >> 10; // Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
            ADCTrigger(CHN7);
    }

    ReadAccelSensors();
    ReadGyroSensors();
    
    // *** Old ARHS
    float g1 = (Gyro.X.value - (float)Gyro.X.error)/(float)FAST_RATE;
    float g2 = (Gyro.Y.value - (float)Gyro.Y.error)/(float)FAST_RATE;
    float g3 = (Gyro.Z.value - (float)Gyro.Z.error)/(float)FAST_RATE;

    float qg1 = -0.5f*(g1*q2 + g2*q3 + g3*q4);
    float qg2 = 0.5f*(g1*q1 + g3*q3 - g2*q4);
    float qg3 = 0.5f*(g2*q1 - g3*q2 + g1*q4);
    float qg4 = 0.5f*(g3*q1 + g2*q2 - g1*q3);
    
    q1 += qg1;
    q2 += qg2;
    q3 += qg3;
    q4 += qg4;
    
    // renormalise using fast inverse sq12re root
    float sumsqu = finvSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q1 *= sumsqu;
    q2 *= sumsqu;
    q3 *= sumsqu;
    q4 *= sumsqu;
    // precalculated values for optimisation
    float q11 = q1 * q1;
    float q12 = q1 * q2;
    float q13 = q1 * q3;
    float q22 = q2 * q2;
    float q23 = q2 * q4;
    float q33 = q3 * q3;
    float q34 = q3 * q4;
    float q44 = q4 * q4;
    float q22Pq33 = q22 + q33;
    float Tq13Mq23 = 2 * (q13 - q23);
    float Tq34Pq12 = 2 * (q34 + q12);
    // avoid gimbal lock at singularity points
    if (Tq13Mq23 == 1) {
        psiAngle = 2 * fatan2(q2, q1);
        thetaAngle = M_PI_2;
        phiAngle = 0;
    }
    else if (Tq13Mq23 == -1) {
        psiAngle = -2 * fatan2(q2, q1);
        thetaAngle = - M_PI_2;
        phiAngle = 0;
    }
    else {
        thetaAngle = fasin(Tq13Mq23);    
        phiAngle = fatan2(Tq34Pq12, (1 - 2*q22Pq33));  
        psiAngle = fatan2((2*(q1 * q4 + q2 * q3)), (1 - 2*(q33 + q44)));  
    }
    // ilink_attitude.roll = phiAngle; //temporarily acquired
    // ilink_attitude.pitch = thetaAngle; //temporarily acquired
    // ilink_attitude.yaw = psiAngle;
    
    // *** Old AHRS drift correction
    float m9 = (q11 - q22 - q33 + q44);
    Gyro.X.verterror = Tq34Pq12*Accel.Z.value - m9*Accel.Y.value;  // Correction vector
    Gyro.Y.verterror = m9*Accel.X.value + Tq13Mq23*Accel.Z.value;
    
    float MdotA = Mag.X.value*Accel.X.value + Mag.Y.value*Accel.Y.value + Mag.Z.value*Accel.Z.value;
    float X = (Mag.X.value - MdotA*Accel.X.value);  // Lagranges Theorum - all of this paragraph resolves the Magneto X and Y axes to the horizontal plane
    float Y = (Mag.Y.value - MdotA*Accel.Y.value);
    
        
    //if(Accel.Z.value > 0.3) {
     Gyro.Z.verterror = psiAngle + fatan2(Y, X); // Heading correction error calculation
    //if (Accel.Z.value < 0) Gyro.Z.verterror = psiAngle - fatan2(Y, X) + M_PI; // Todo: fix upside down case

    if (Gyro.Z.verterror > M_PI) Gyro.Z.verterror -= M_TWOPI;
    if (Gyro.Z.verterror < -M_PI) Gyro.Z.verterror += M_TWOPI;
        

        
    Gyro.X.error = -DRIFT_AccelKp*Gyro.X.verterror;
    Gyro.Y.error = -DRIFT_AccelKp*Gyro.Y.verterror;
    
    
    if(throttle == 0) {
        Gyro.Z.error = 100.0*DRIFT_MagKp*Gyro.Z.verterror;
    }

    else if(CFDC_count == 10) {
        Gyro.Z.error = 10.0*DRIFT_MagKp*Gyro.Z.verterror;
    }
    else {
        Gyro.Z.error = DRIFT_MagKp*Gyro.Z.verterror;
    }
        
    // *** STATE MACHINE
        float pitchDemandSpin = 0;
        float rollDemandSpin = 0;
        
        float pitcherror, rollerror, yawerror;
        float pitchcorrection, rollcorrection, yawcorrection;
        
        // *** Flight Control
        // Control Inputs
        
        if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
            if(rxLoss < 50) {
                if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
                    if(rcInput[RX_ELEV] > MAXTHRESH) {
                        // arm
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;
                            Arm();
                        }
                    }
                    else if(rcInput[RX_ELEV] < MINTHRESH) {
                        // disarm
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;
                            Disarm();
                        }
                    }
                    else {
                        zeroThrotCounter = 0;
                    }
                }
                else if(rcInput[RX_ELEV] < MAXTHRESH && rcInput[RX_ELEV] > MINTHRESH) {
                    if(rcInput[RX_AILE] > MAXTHRESH) {
                        // calib gyro    
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;
                            CalibrateGyro();
                        }
                    }
                    else if(rcInput[RX_AILE] < MINTHRESH) {
                        // calib magneto
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;
                            CalibrateMagneto();
                        }
                    }
                }
                
            }
            else {
                zeroThrotCounter = 0;
            }
            
            
            
        }
        else {
            float tempf = 0;
            if (throttle > 350) yaw_lock++;
            if (yaw_lock > 2001) yaw_lock = 2001;
            
            if(MODE_AUTO) {
                pitch.demandtemp = ilink_attitude_rx.pitch;
                roll.demandtemp = ilink_attitude_rx.roll;
                tempf = ilink_attitude_rx.yaw;
            }
            else {
                pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
                roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
                
                if ((CFDC_count == 10) || (yaw_lock > 2000)) {
                    tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; //3125
                }
            }
            
			
            
            
            
            if(pitch.demandtemp > pitch.demand) {
                if(pitch.demandtemp - pitch.demand > LIM_RATE/(float)FAST_RATE && pitch.demand > 0) pitch.demand = pitch.demand + LIM_RATE/(float)FAST_RATE;
                else pitch.demand = pitch.demandtemp;
            }
            else {
                if(pitch.demand - pitch.demandtemp > LIM_RATE/(float)FAST_RATE && pitch.demand < 0) pitch.demand = pitch.demand - LIM_RATE/(float)FAST_RATE;
                else pitch.demand = pitch.demandtemp;
            }
            
            if(roll.demandtemp > roll.demand) {
                if(roll.demandtemp - roll.demand > LIM_RATE/(float)FAST_RATE && roll.demand > 0) roll.demand = roll.demand + LIM_RATE/(float)FAST_RATE;
                else roll.demand = roll.demandtemp;
            }
            else {
                if(roll.demand - roll.demandtemp > LIM_RATE/(float)FAST_RATE && roll.demand < 0) roll.demand = roll.demand - LIM_RATE/(float)FAST_RATE;
                else roll.demand = roll.demandtemp;
            }
            
            //pitch.demand = pitch.demandav;
            //roll.demand = roll.demandav;
            // Yaw
            
            
            if(fabsf(tempf) > YAW_DEADZONE || MODE_AUTO) {
                yaw.demand += tempf;
            }
            
            if(yaw.demand > M_PI) {
                yaw.demand -= M_TWOPI;
                yaw.demandOld -= M_TWOPI;
            }
            else if(yaw.demand < -M_PI) {
                yaw.demand += M_TWOPI;
                yaw.demandOld += M_TWOPI;
            }

            lat_diff = (double)(ilink_position.targetX - ilink_position.craftX) * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth and deg-rad conversion: 6371000*PI()/180
            lon_diff = (double)(ilink_position.targetY - ilink_position.craftY) * (double)111194.92664455873734580834 * fcos((float)((double)ilink_position.craftX*(double)0.01745329251994329577)); // 0.01745329251994329577f is deg-rad conversion PI()/180
            alt_diff = (double)(ilink_position.targetZ - ilink_position.craftZ) + GPS_ALTHo;
                    
            if (MODE_GPS == 1) {
                    if(ilink_position.isNew) {
                        ilink_position.isNew = 0;
                        
                        lat_diff_i += lat_diff;
                        lon_diff_i += lon_diff;
                        lat_diff_i *= GPS_Kde;
                        lon_diff_i *= GPS_Kde;
                        alt_diff_i += alt_diff;
                        alt_diff_i *= GPS_ALTKde;
                        
                        lat_diff_d = lat_diff - lat_diff_old;
                        lat_diff_old = lat_diff;
                        lon_diff_d = lon_diff - lon_diff_old;
                        lon_diff_old = lon_diff;
                        alt_diff_d = alt_diff - alt_diff_old;
                        alt_diff_old = alt_diff;
                    }
                    
                    
                    pitch.demand = GPS_Kp*lat_diff + GPS_Ki*lat_diff_i + GPS_Kd*lat_diff_d;
                    roll.demand = GPS_Kp*lon_diff + GPS_Ki*lon_diff_i + GPS_Kd*lon_diff_d;
                    
                    pitchDemandSpin = fsin(-psiAngle+M_PI_2)*pitch.demand - fsin(-psiAngle)*roll.demand;
                    rollDemandSpin = fsin(-psiAngle)*pitch.demand + fsin(-psiAngle+M_PI_2)*roll.demand;
                    
                    gpsThrottle = GPS_ALTKp * alt_diff + GPS_ALTKi * alt_diff_i + GPS_ALTKd * alt_diff_d + alt_throttle;
                    throttle = gpsThrottle;
                    
            }
            else {
        
                lat_diff_i = 0;
                lon_diff_i = 0;
                lat_diff_old = lat_diff;
                lon_diff_old = lon_diff;
                lat_diff_d = 0;
                lon_diff_d = 0;
        
        
                alt_diff_i = 0;
                alt_diff_old = alt_diff;
                alt_diff_d = 0;
                alt_throttle = throttle;
        
                if (MODE_SIMP == 1) {
                    pitchDemandSpin = fsin(-psiAngle+psiAngleinit+M_PI_2 + 0.78539816339744830962)*pitch.demand - fsin(-psiAngle+psiAngleinit + 0.78539816339744830962)*roll.demand;
                    rollDemandSpin = fsin(-psiAngle+psiAngleinit + 0.78539816339744830962)*pitch.demand + fsin(-psiAngle+psiAngleinit+M_PI_2 + 0.78539816339744830962)*roll.demand;
                }
                else {
                    // cross 0 simplicity 0
                    pitchDemandSpin = fsin(0.78539816339744830962+M_PI_2)*pitch.demand - fsin(0.78539816339744830962)*roll.demand;
                    rollDemandSpin = fsin(0.78539816339744830962)*pitch.demand + fsin(0.78539816339744830962+M_PI_2)*roll.demand;
                }
            }
        }
    
        if(pitchDemandSpin > LIM_ANGLE) pitchDemandSpin = LIM_ANGLE;		//limit roll and pitch angles
        if(pitchDemandSpin < -LIM_ANGLE) pitchDemandSpin = -LIM_ANGLE;
        if(rollDemandSpin > LIM_ANGLE) rollDemandSpin = LIM_ANGLE;
        if(rollDemandSpin < -LIM_ANGLE) rollDemandSpin = -LIM_ANGLE;
        
 // *** PID
        pitch.derivative = (pitchDemandSpin - pitch.demandOld);    
        roll.derivative = (rollDemandSpin - roll.demandOld);
        yaw.derivative = (yaw.demand - yaw.demandOld);
        
        pitch.demandOld = pitchDemandSpin;
        roll.demandOld = rollDemandSpin;
        yaw.demandOld = yaw.demand;
        
        pitcherror = pitchDemandSpin + thetaAngle;
        rollerror = rollDemandSpin - phiAngle;
        yawerror = yaw.demand + psiAngle; 
        
        if(pitcherror > M_PI) pitcherror -= M_TWOPI;
        else if(pitcherror < -M_PI) pitcherror += M_TWOPI;
        
        if(rollerror > M_PI) rollerror -= M_TWOPI;
        else if(rollerror < -M_PI) rollerror += M_TWOPI;
        
        if(yawerror > M_PI) yawerror -= M_TWOPI;
        else if(yawerror < -M_PI) yawerror += M_TWOPI;
        
        
        if(throttle > 350) {
            pitch.integral += pitcherror;
            roll.integral += rollerror;
        }
        else {
            pitch.integral += pitcherror/5.0f;
            roll.integral += rollerror/5.0f;
        }
        
        pitch.integral *= PITCH_De;
        roll.integral *= ROLL_De;

        
        // Detune at high throttle
        float throttlefactor = throttle/MAXSTICK;
        if(throttlefactor > 1) throttlefactor = 1;
        
        float detunefactor = 1-(throttlefactor * DETUNE);
        // float thisPITCH_Kd = PITCH_Kd;
        // float thisPITCH_Kdd = PITCH_Kdd * detunefactor;
        // float thisROLL_Kd = ROLL_Kd;
        // float thisROLL_Kdd = ROLL_Kdd * detunefactor;
        // float thisPITCH_Kd = PITCH_Kd * detunefactor;
        // float thisPITCH_Kdd = PITCH_Kdd * detunefactor;
        // float thisROLL_Kd = ROLL_Kd * detunefactor;
        // float thisROLL_Kdd = ROLL_Kdd * detunefactor;
        float thisPITCH_Ki = PITCH_Ki;
        float thisROLL_Ki = ROLL_Ki;
		float thisPITCH_Kd = PITCH_Kd;
        float thisPITCH_Kdd = PITCH_Kdd;
        float thisROLL_Kd = ROLL_Kd;
        float thisROLL_Kdd = ROLL_Kdd;
        
        
        // Attitude control PID loops
        
        pitchcorrection = -((float)Gyro.Y.value - PITCH_Boost*pitch.derivative) * thisPITCH_Kd;
        pitchcorrection += -thisPITCH_Kdd*((float)Gyro.Y.value - pitch.valueOld);
        pitchcorrection += -PITCH_Kp*pitcherror;
        pitchcorrection += -thisPITCH_Ki*pitch.integral;

        rollcorrection = -((float)Gyro.X.value - ROLL_Boost*roll.derivative) * thisROLL_Kd;
        rollcorrection += -thisROLL_Kdd*((float)Gyro.X.value - roll.valueOld);
        rollcorrection += ROLL_Kp*rollerror;
        rollcorrection += thisROLL_Ki*roll.integral;

        yawcorrection = -((float)Gyro.Z.value + YAW_Boost*yaw.derivative) * YAW_Kd; 
        yawcorrection += -YAW_Kp*yawerror;
        
        pitch.valueOld = (float)Gyro.Y.value;
        roll.valueOld = (float)Gyro.X.value;
        yaw.valueOld = (float)Gyro.Z.value;
        
        motorN = pitchcorrection;
        motorE = -rollcorrection;
        motorS = -pitchcorrection;
        motorW = rollcorrection;
        
        // float motorNdelta = pitchcorrectionav - motorN; 
        // float motorEdelta = -rollcorrectionav - motorE;
        // float motorSdelta = -pitchcorrectionav - motorS;
        // float motorWdelta = rollcorrectionav - motorW;
        
        // if(motorNdelta > 0) motorN += motorNdelta * ADJ_UP;
        // else motorN += motorNdelta * ADJ_DOWN;

        // if(motorEdelta > 0) motorE += motorEdelta * ADJ_UP;
        // else motorE += motorEdelta * ADJ_DOWN;

        // if(motorSdelta > 0) motorS += motorSdelta * ADJ_UP;
        // else motorS += motorSdelta * ADJ_DOWN;

        // if(motorWdelta > 0) motorW += motorWdelta * ADJ_UP;
        // else motorW += motorWdelta * ADJ_DOWN;
        
        motorN -= yawcorrection;
        motorE += yawcorrection;
        motorS -= yawcorrection;
        motorW += yawcorrection;
        
        
        motorNav *= SPR_OUT;
        motorNav += (1-SPR_OUT) * motorN;
		
		motorEav *= SPR_OUT;
        motorEav += (1-SPR_OUT) * motorE;
		
		motorSav *= SPR_OUT;
        motorSav += (1-SPR_OUT) * motorS;
		
		motorWav *= SPR_OUT;
        motorWav += (1-SPR_OUT) * motorW;
        
        
        
        
        
        
        ////////// ULTRASOUND CODE ////////////////////////////////////////////////////

				// If airborne is not equal to 1
				if(airborne == 0) {
					
					// We use Manual Throttle to lift off
					throttle = rcInput[RX_THRO]  - throttletrim;
					alt.demandincr =  0;
					
					// If someone increases the throttle violently, turn the motors back off
					// Violent throttle movements cause the craft to enter the ultrasound region at too high a velocity
					if((throttle - throttleold > LIM_THROT) && (MODE_ULTRA ==1)) {
						throttle = throttleold + LIM_THROT;
					}
					throttleold = throttle;
																
				}
				

				if((alt.value > ULTRA_TKOFF) && (airborne == 0)) { 
					// just taken off, set airborne to 1 and remember takeoff throttle
					airborne = 1;
					ultraTkOffThrottle = throttle;
					alt.demandincr = 0;					
				}
			
		
				// Create a deadzone around throttle at lift off, if new stick position is higher than this deadzone then increase altitude demand and vice versa		
				if (((((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffThrottle) > ULTRA_DEAD) || ((((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffThrottle) < -ULTRA_DEAD)) {
                    ilink_attitude.roll = alt.demandincr;
                    ilink_attitude.pitch = (float)rcInput[RX_THRO] - (float) throttletrim;
                    ilink_attitude.yaw = ultraTkOffThrottle;
                    ilink_attitude.pitchRate = (ULTRA_DRMP/FAST_RATE);

                    alt.demandincr =  alt.demandincr + (((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffThrottle)*(ULTRA_DRMP/FAST_RATE);
				
                
                    ilink_attitude.rollRate = alt.demandincr;
                }
                

				


				if(alt.demandincr < -LIM_ALT) alt.demandincr = -LIM_ALT;
				if(alt.demandincr > LIM_ALT) alt.demandincr = LIM_ALT;


				alt.demand = ULTRA_TKOFF + alt.demandincr;
				
				
			
				if ((ultra > 0) && (ultra < ULTRA_LND) && (airborne == 1)) {
					// If valid ultrasound reading and reading is less than landing threshold and we are airborne
					// then increase landing counter
					landing++;
					
					//If the above is satisfied consecutively more than ULTRA_DTCT times then shut down the motors
					if(landing > ULTRA_DTCT) { 
						airborne = 0;
						throttleHoldOff = 1;
						throttle = 0;
						alt.demandincr =  0;
						yaw_lock = 0;						
					}
					
				}	
				// If consecutive run of readings is broken, reset the landing counter.
				else {
					landing = 0;
				}
				
				
				
	
				alt.error = alt.demand - alt.value;
                
				alt.derivative = alt.value - alt.valueOld;
				
					

				alt.integral *= ULTRA_De;
				alt.integral += alt.error;


				if ((ultraLoss >= ULTRA_OVTH) && (throttle > 200)) {   // when consective ultrasound altitude data losses rise above ULTRA_OVTH, throttle level is reduced
						
					ultrathrottle -= ULTRA_OVDEC;  //Decays throttle to ensure craft returns to ground level
						
				}	
				else {
				ultrathrottle = alt.derivative * -ULTRA_Kd;
				ultrathrottle += ULTRA_Kp*alt.error;
				ultrathrottle += ULTRA_Ki*alt.integral;
				}		
				
				
				if ((MODE_ULTRA == 1) && (airborne == 1)){
					throttle = ultraTkOffThrottle + ultrathrottle;
				}
                
               
			
			//////////////////////////////////////////////////////////////////////////////////////

        
        
        
        
        
        if ((MODE_ULTRA == 0) && (MODE_GPS == 0)) {
            throttle = rcInput[RX_THRO] - throttletrim;
            alt.demandincr = 0;
        }
        
        
        
        if (throttleHoldOff > 0) {
            throttle = 0;
            alt.demandincr =  0;
            airborne = 0;
            
        }
        
        
        if (throttle < 0) throttle = 0;
        
        
        
        
        
        tempN = (signed short)motorNav + (signed short)throttle + THROTTLEOFFSET;
        tempE = (signed short)motorEav + (signed short)throttle + THROTTLEOFFSET;
        tempS = (signed short)motorSav + (signed short)throttle + THROTTLEOFFSET;
        tempW = (signed short)motorWav + (signed short)throttle + THROTTLEOFFSET;
        
        if (rcInput[RX_THRO] - throttletrim <  OFFSTICK || throttleHoldOff > 0 || rxLoss > 25) {
            pitch.integral=0;
            roll.integral=0;
            alt.integral = 0;
            
            throttle = 0;
            airborne = 0;
            alt.demandincr =  0;
			yaw_lock = 0;
            
            
            if(rcInput[RX_THRO] - throttletrim <  OFFSTICK) throttleHoldOff = 0;
            
            yaw.demand = -psiAngle;
            yaw.demandOld = -psiAngle;
            
            if(armed) {
                PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
            }
            ilink_outputs0.channel[0] = THROTTLEOFFSET;
            ilink_outputs0.channel[1] = THROTTLEOFFSET;
            ilink_outputs0.channel[2] = THROTTLEOFFSET;
            ilink_outputs0.channel[3] = THROTTLEOFFSET;
            
            motorN = 0;
            motorE = 0;
            motorS = 0;
            motorW = 0;
			
			motorNav = 0;
            motorEav = 0;
            motorSav = 0;
            motorWav = 0;
        }
        else if(armed) {
            float temp;
            
            if(throttle > MAXTHROTTLE*MAXTHROTTLEPERCENT) throttle = MAXTHROTTLE*MAXTHROTTLEPERCENT;
            
            temp = tempN;
            if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
            else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
            PWMSetN(temp);
            ilink_outputs0.channel[0] = temp;
            
            temp = tempE;
            if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
            else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
            PWMSetE(temp);
            ilink_outputs0.channel[1] = temp;
            
            temp = tempS;
            if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
            else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
            PWMSetS(temp);
            ilink_outputs0.channel[2] = temp;
            
            temp = tempW;
            if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
            else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
            PWMSetW(temp);
            ilink_outputs0.channel[3] = temp;
            ilink_outputs0.isNew = 1;

        }
            
		// *** Gimbal
		float pitch_temp = fsin(0.78539+M_PI_2)*thetaAngle - fsin(0.78539)*phiAngle;
		float roll_temp = fsin(0.78539)*thetaAngle + fsin(0.78539+M_PI_2)*phiAngle;
		
		float value =  GIM_RMID + (float) roll_temp * GIM_RSCAL / M_PI;
	
		if(value > GIM_RLIMH) value = GIM_RLIMH;
		if(value < GIM_RLIML) value = GIM_RLIML;

		PWMSetX(value);
		ilink_outputs0.channel[4] = value;
		
		value = GIM_PMID + (float)pitch_temp * GIM_PSCAL / M_PI + gim_ptemp * GIM_PPLUS;
		if(value > GIM_PLIMH) value = GIM_PLIMH;
		if(value < GIM_PLIML) value = GIM_PLIML;
		PWMSetY(value);
		ilink_outputs0.channel[5] = value;
			
     
}

void ReadGyroSensors(void) { // 400Hz -ish
    signed short data[4];
    if(GetGyro(data)) {
        Gyro.X.raw = data[0];
        Gyro.Y.raw = data[1];
        Gyro.Z.raw = data[2];
        ilink_rawimu.xGyro = Gyro.X.raw;
        ilink_rawimu.yGyro = Gyro.Y.raw;
        ilink_rawimu.zGyro = Gyro.Z.raw;
        Gyro.X.total -= Gyro.X.history[Gyro.count];
        Gyro.Y.total -= Gyro.Y.history[Gyro.count];
        Gyro.Z.total -= Gyro.Z.history[Gyro.count];
        Gyro.X.total += Gyro.X.raw;
        Gyro.Y.total += Gyro.Y.raw;
        Gyro.Z.total += Gyro.Z.raw;
        Gyro.X.history[Gyro.count] = Gyro.X.raw;
        Gyro.Y.history[Gyro.count] = Gyro.Y.raw;
        Gyro.Z.history[Gyro.count] = Gyro.Z.raw;
        Gyro.X.av = (float)Gyro.X.total/(float)GAV_LEN;
        Gyro.Y.av = (float)Gyro.Y.total/(float)GAV_LEN;
        Gyro.Z.av = (float)Gyro.Z.total/(float)GAV_LEN;
        if(++Gyro.count >= GAV_LEN) Gyro.count = 0;
        
        // Calculate Gyro
        // scale factors: 2000dps: 818.51113590117601252569
        // 500dps: 3274.04454360470405010275
        // 250dps: 6548.0890872094081002055
        Gyro.X.value = (Gyro.X.av - Gyro.X.offset)/818.51113590117601252569f;
        Gyro.Y.value = (Gyro.Y.av - Gyro.Y.offset)/818.51113590117601252569f;
        Gyro.Z.value = (Gyro.Z.av - Gyro.Z.offset)/818.51113590117601252569f;
        ilink_scaledimu.xGyro = Gyro.X.value * 1000;
        ilink_scaledimu.yGyro = Gyro.Y.value * 1000;
        ilink_scaledimu.zGyro = Gyro.Z.value * 1000;
    }
}

void ReadAccelSensors(void) {
    float sumsqu;
    signed short data[4];
    if(GetAccel(data)) {
        Accel.X.raw = data[0];
        Accel.Y.raw = data[1];
        Accel.Z.raw = data[2];
        ilink_rawimu.xAcc = Accel.X.raw; //temporarily repurposed
        ilink_rawimu.yAcc = Accel.Y.raw; //temporarily repurposed
        ilink_rawimu.zAcc = Accel.Z.raw; //temporarily repurposed
        Accel.X.total -= Accel.X.history[Accel.count];
        Accel.Y.total -= Accel.Y.history[Accel.count];
        Accel.Z.total -= Accel.Z.history[Accel.count];
        Accel.X.total += Accel.X.raw;
        Accel.Y.total += Accel.Y.raw;
        Accel.Z.total += Accel.Z.raw;
        Accel.X.history[Accel.count] = Accel.X.raw;
        Accel.Y.history[Accel.count] = Accel.Y.raw;
        Accel.Z.history[Accel.count] = Accel.Z.raw;
        Accel.X.av = (float)Accel.X.total/(float)AAV_LEN;
        Accel.Y.av = (float)Accel.Y.total/(float)AAV_LEN;
        Accel.Z.av = (float)Accel.Z.total/(float)AAV_LEN;
        if(++Accel.count >= AAV_LEN) Accel.count = 0;
        
        // Normalise accelerometer
        sumsqu = finvSqrt((float)Accel.X.av*(float)Accel.X.av + (float)Accel.Y.av*(float)Accel.Y.av + (float)Accel.Z.av*(float)Accel.Z.av); // Accelerometr data is normalised so no need to convert units.
        Accel.X.value = (float)Accel.X.av * sumsqu;
        Accel.Y.value = (float)Accel.Y.av * sumsqu;
        Accel.Z.value = (float)Accel.Z.av * sumsqu;
        ilink_scaledimu.xAcc = Accel.X.value * 1000;
        ilink_scaledimu.yAcc = Accel.Y.value * 1000;
        ilink_scaledimu.zAcc = Accel.Z.value * 1000;
    }
}

void ReadMagSensors(void) { // 50Hz-ish
    float sumsqu, temp1, temp2, temp3;
    signed short data[4];
    if(GetMagneto(data)) {
        Mag.X.raw = data[0];
        Mag.Y.raw = data[1];
        Mag.Z.raw = data[2];

        Mag.X.total -= Mag.X.history[Mag.count];
        Mag.Y.total -= Mag.Y.history[Mag.count];
        Mag.Z.total -= Mag.Z.history[Mag.count];
        Mag.X.total += Mag.X.raw;
        Mag.Y.total += Mag.Y.raw;
        Mag.Z.total += Mag.Z.raw;
        Mag.X.history[Mag.count] = Mag.X.raw;
        Mag.Y.history[Mag.count] = Mag.Y.raw;
        Mag.Z.history[Mag.count] = Mag.Z.raw;
        Mag.X.av = (float)Mag.X.total/(float)MAV_LEN;
        Mag.Y.av = (float)Mag.Y.total/(float)MAV_LEN;
        Mag.Z.av = (float)Mag.Z.total/(float)MAV_LEN;
		if(++Mag.count >= MAV_LEN) Mag.count = 0;
		
		ilink_rawimu.xMag = Mag.X.av;
        ilink_rawimu.yMag = Mag.Y.av;
        ilink_rawimu.zMag = Mag.Z.av;
		
		if (CFDC_count == 10) {
			Mag.X.av += CFDC_mag_x_error;
			Mag.Y.av += CFDC_mag_y_error;
			Mag.Z.av += CFDC_mag_z_error;
		}

        
        // Correcting Elipsoid Centre Point
        temp1 = Mag.X.av - MAGCOR_M1;
        temp2 = Mag.Y.av - MAGCOR_M2;
        temp3 = Mag.Z.av - MAGCOR_M3;

        // Reshaping Elipsoid to Sphere
        temp1 = MAGCOR_N1 * temp1 + MAGCOR_N2 * temp2 + MAGCOR_N3 * temp3;
        temp2 = MAGCOR_N5 * temp2 + MAGCOR_N6 * temp3;
        temp3 = MAGCOR_N9 * temp3;
		
        //wtf
		/*
		ilink_rawimu.xAcc = throttle; // temporarily requisitioned
		ilink_rawimu.yAcc = airborne; // temporarily requisitioned
		*/

		
		
		/////////////Current Field Distortion Compensation (CFDC)
		
		if (throttle == 0) {
			CFDC_mag_x_zero = Mag.X.av;
			CFDC_mag_y_zero = Mag.Y.av;
			CFDC_mag_z_zero = Mag.Z.av;
			CFDC_count = 0;
			
			
		}
		
		
		if ((airborne == 1)  && (CFDC_count < 10)){
		
			CFDC_count ++;         
            
            CFDC_mag_x_error = CFDC_mag_x_error + CFDC_mag_x_zero - Mag.X.av;
			CFDC_mag_y_error = CFDC_mag_y_error + CFDC_mag_y_zero - Mag.Y.av;
			CFDC_mag_z_error = CFDC_mag_z_error + CFDC_mag_z_zero - Mag.Z.av;
			
			if (CFDC_count == 10)	{
				CFDC_mag_x_error /= 10;
				CFDC_mag_y_error /= 10;
				CFDC_mag_z_error /= 10;
			}
			
		}
		
		
		///////////////////////////////////////////////////////////////////

        		// Normalize magneto
        sumsqu = finvSqrt((float)temp1*(float)temp1 + (float)temp2*(float)temp2 + (float)temp3*(float)temp3); // Magnetoerometr data is normalised so no need to convert units.
        Mag.X.value = (float)temp1 * sumsqu;
        Mag.Y.value = (float)temp2 * sumsqu;
        Mag.Z.value = (float)temp3 * sumsqu;
		
		
        
		
		
		
    }
    

}

// ****************************************************************************
// *** EEPROM Functions
// ****************************************************************************

// *** This function loads all parameters from EEPROM.  First it loads the
// parameters into temporary storage to verify the checksum.  Only if the
// checksums are correct will the function update the parameters in RAM.
void EEPROMLoadAll(void) {
    unsigned char chkA, chkB;
    unsigned int i;
    unsigned char * ptr;
    float tempStorage[EEPROM_MAX_PARAMS+1];
    
    // Read EEPROM into some temporarily allocated space
    EEPROMRead(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4 + 2);
    
    // Calculate the Fletcher-16 checksum
    chkA=EEPROM_VERSION;
    chkB=EEPROM_VERSION;
    ptr = (unsigned char *)&tempStorage;
    for(i=0; i<paramCount*4; i++) {
        chkA += ptr[i];
        chkB += chkA;
    }
    
    // Verify the checksum is valid (at this point i points to the correct elements for chkA and chkB)
    if(chkA == ptr[i] && chkB == ptr[i+1]) {
        // For valid data, load into parameters in RAM
        for(i=0; i<paramCount; i++) {
            paramStorage[i].value = tempStorage[i];
        }
    }
}

// *** This function saves all parameters to EEPROM.
void EEPROMSaveAll(void) {
    unsigned char chkA, chkB;
    unsigned int i;
    unsigned char * ptr;
    float tempStorage[EEPROM_MAX_PARAMS+1];
    
    // Save parameters into temporary space
    
    chkA=EEPROM_VERSION;
    chkB=EEPROM_VERSION;
    for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
        tempStorage[i] = paramStorage[i].value;
        ptr = (unsigned char *)&(tempStorage[i]);
        chkA += ptr[0];
        chkB += chkA;
        chkA += ptr[1];
        chkB += chkA;
        chkA += ptr[2];
        chkB += chkA;
        chkA += ptr[3];
        chkB += chkA;
    }
    
    ptr = (unsigned char *)&(tempStorage[i]);
    ptr[0] = chkA;
    ptr[1] = chkB;
    
    EEPROMWrite(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4 + 2);
}

// ****************************************************************************
// *** Other Functions
// ****************************************************************************

// *** nonlinearises throttle
/*
float nonLinearThrottle(float input) {
    float output;
	
    if (input < 10*THROTTLE_X1) {
		output = input * THROTTLE_Y1;
        output /= THROTTLE_X1;
	}
    else if(input < 10*THROTTLE_X2) {
		output = (input-(THROTTLE_X1*10));
		output *= (THROTTLE_Y2-THROTTLE_Y1);
		output /= (THROTTLE_X2-THROTTLE_X1);
		output += (10*THROTTLE_Y1);
	}
    else {
        output = (THROTTLE_Yn-THROTTLE_Y2);
		output *= (input-(10*THROTTLE_X2));
		output /= (100-THROTTLE_X2);
		output += (10*THROTTLE_Y2);
	}
    return output;
}*/


// ****************************************************************************
// *** Communications
// ****************************************************************************

// *** Deal with ilink requets
void ILinkMessageRequest(unsigned short id) {
    unsigned short * ptr = 0;
    unsigned short maxlength = 0;
    
    switch(id) {
        case ID_ILINK_IDENTIFY:     ptr = (unsigned short *) &ilink_identify;   maxlength = sizeof(ilink_identify)/2 - 1;   break;
        case ID_ILINK_THALSTAT:     ptr = (unsigned short *) &ilink_thalstat;   maxlength = sizeof(ilink_thalstat)/2 - 1;   break;
        case ID_ILINK_RAWIMU:       ptr = (unsigned short *) &ilink_rawimu;     maxlength = sizeof(ilink_rawimu)/2 - 1;     break;
        case ID_ILINK_SCALEDIMU:    ptr = (unsigned short *) &ilink_scaledimu;  maxlength = sizeof(ilink_scaledimu)/2 - 1;  break;
        case ID_ILINK_ALTITUDE:     ptr = (unsigned short *) &ilink_altitude;   maxlength = sizeof(ilink_altitude)/2 - 1;   break;
        case ID_ILINK_ATTITUDE:     ptr = (unsigned short *) &ilink_attitude;   maxlength = sizeof(ilink_attitude)/2 - 1;   break;
        case ID_ILINK_INPUTS0:
            if(ilink_inputs0.isNew) {
                ilink_inputs0.isNew = 0;
                ptr = (unsigned short *) &ilink_inputs0;
                maxlength = sizeof(ilink_inputs0)/2 - 1;
            }
            break;
        case ID_ILINK_OUTPUTS0:     ptr = (unsigned short *) &ilink_outputs0;   maxlength = sizeof(ilink_outputs0)/2 - 1;   break;
        case ID_ILINK_CLEARBUF:
            FUNCILinkTxBufferPushPtr = 0;
            FUNCILinkTxBufferPopPtr = 0;
            break;
    }

    if(ptr) {
        ILinkSendMessage(id, ptr, maxlength);
    }
}

// *** iLink interrupt handler
void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
    unsigned short * ptr = 0;
    unsigned int i, j;
    
    switch(id) {
        case ID_ILINK_THALPAREQ: ptr = (unsigned short *) &ilink_thalpareq; break;
        case ID_ILINK_THALPARAM: ptr = (unsigned short *) &ilink_thalparam_rx; break;
        case ID_ILINK_THALCTRL: ptr = (unsigned short *) &ilink_thalctrl; break;
        case ID_ILINK_POSITION: ptr = (unsigned short *) &ilink_position; break;
        case ID_ILINK_ATTITUDE: ptr = (unsigned short *) &ilink_attitude_rx; LEDToggle(PLED); break;
    }
    
    if(ptr) {
        for(i=0; i<length; i++) {
            ptr[i] = buffer[i];
        }
        ptr[i] = 1; // this should be the isNew byte
    }
    
    switch(id) {
        case ID_ILINK_THALPAREQ:
            ilink_thalpareq.isNew = 0;
            switch(ilink_thalpareq.reqType) {
                case 1: // get one
                    if(ilink_thalpareq.paramID == 0xffff) {

                        for (i=0; i<paramCount; i++){
                            unsigned char match = 1;
                            for (j=0; j<16; j++) {
                                if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {

                                    match = 0;
                                    break;
                                }
                                if (paramStorage[i].name[j] == '\0') break;
                            }
                            
                            if(match == 1) {
                                // when a match is found get the iD
                                paramSendCount = i;
                                paramSendSingle = 1;
                                break;
                            }
                        }
                    }


                    else {
                        paramSendCount = ilink_thalpareq.paramID;
                        paramSendSingle = 1;
                    }
                    break;
                case 2: // save all
                    EEPROMSaveAll();
                    ilink_thalpareq.isNew = 1;
                    ilink_thalctrl.command = MAVLINK_MSG_ID_COMMAND_LONG;
                    ilink_thalctrl.data = MAV_CMD_PREFLIGHT_STORAGE;
                    ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl, sizeof(ilink_thalctrl)/2 - 1);
                    break;
                case 3: // reload all
                    EEPROMLoadAll();
                    ilink_thalpareq.isNew = 1;
                    ilink_thalctrl.command = MAVLINK_MSG_ID_COMMAND_LONG;
                    ilink_thalctrl.data = MAV_CMD_PREFLIGHT_STORAGE;
                    ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl, sizeof(ilink_thalctrl)/2 - 1);
                    // fall through to get all
                default:
                case 0: // get all
                    paramSendCount = 0;
                    paramSendSingle = 0;
                    break;
                }
            break;
            
        case ID_ILINK_THALCTRL:
            switch(ilink_thalctrl.command) {
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    switch(ilink_thalctrl.data) {
                        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            // for some reason qGroundControl uses this message for KILL UAS
                            Disarm();
                            ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl, sizeof(ilink_thalctrl)/2 - 1);
                            break;
                    }
                    break;
                case MAVLINK_MSG_ID_SET_MODE:
                    if(ilink_thalctrl.data & MAV_MODE_FLAG_SAFETY_ARMED) {
                        Arm();
                    }
                    else {
                        Disarm();
                    }
                    
                    if(ilink_thalctrl.data & MAV_MODE_FLAG_GUIDED_ENABLED) {
                        MODE_GPS = 1;
                        ilink_thalstat.flightMode |= (0x1 << 4);
                    }
                    else {                                        
                        MODE_GPS = 0;
                        ilink_thalstat.flightMode &= ~(0x1 << 4);
                    }
                    
                    break;
            }
            break;
        case ID_ILINK_THALPARAM:
            // match up received parameter with stored parameter.  MAVLink y u no give index?
            for (i=0; i<paramCount; i++){
                unsigned char match = 1;
                for (j=0; j<16; j++) {
                    if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {
                        match = 0;
                        break;
                    }
                    if (paramStorage[i].name[j] == '\0') break;
                }
                
                if(match == 1) {
                    // when a match is found, save it to paramStorage
                    paramStorage[i].value = ilink_thalparam_rx.paramValue;
                    
                    // then order the value to be sent out again using the param send engine
                    // but deal with cases where it's already in the process of sending out data
                    if(paramSendCount < paramCount) {
                        // parameter engine currently sending out data
                        if(paramSendCount >= i) {
                            // if parameter engine already sent out this now-changed data, redo this one, otherwise no action needed
                            paramSendCount = i;
                        }
                    }
                    else {
                        // parameter engine not currently sending out data, so send single parameter
                        paramSendCount = i;
                        paramSendSingle = 1;
                    }
                    break;
                }
            }
            break;
    }
}