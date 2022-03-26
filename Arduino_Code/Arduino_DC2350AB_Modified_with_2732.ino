/************************* Includes ***************************/
#include <Arduino.h>
#include <stdint.h>
#include <ltcmuc_tools.h>
#include <SPI.h>
#include <Wire.h>
#include <LTC2949.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6813.h"


/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2
#define LTCDEF__CS 10
#define LTCDEF_CELL_MONITOR_COUNT 1
/************************* Pins *****************************/
int THERMISTOR_PIN = A0; 
int AMS_SWITCH = A1;

const byte FAULT_LED = 3;
const byte Traction_LED_PIN = 4;
const byte AMS_LED_PIN = 6;
const byte Interlock_Positive_Pin = 7;
const byte Interlock_Negative_Pin = 5;


/**************** Local Function Declaration *******************/
void measurement_loop(uint8_t datalog_en);
void print_menu(void);


void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat(void);



void print_overlap_results(int8_t error);

void print_open_wires(void);
int8_t select_s_pin(void);


void serial_print_hex(uint8_t data);
void print_wrcomm(void);
void print_rxcomm(void);


float A_1 =  0.5415349602e-3, B_1 = 2.650202585e-4, C_1 = -0.4340516974e-7; //50k thermistors 
float A =  0.8300937026e-3,B = 2.098161925e-4, C = 0.7098552508e-7; //100k thermistors
byte pins[] = {
             0b00000001, //#1
             0b00000010, //#2
             0b00000100, //#3
             0b00001000, //#4
             0b00010000, //#5
             0b00100000, //#6
             0b01000000, //#7
             0b10000000, //#8
};
             
/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 1;//!< Number of ICs in the daisy chain

/********************************************************************
 ADC Command Configurations. See LTC681x.h for options
*********************************************************************/
const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted 
const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 42000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.2V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = ENABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = ENABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t OPEN_WIRE_LOOP = DISABLED; //!< This is to ENABLED or DISABLED open wire checking in a continuous loop
const uint8_t DC2732 = ENABLED; //!<Enable data communication with DC2732 fuel gauge
/************************************
  END SETUP
*************************************/

/*******************************************************
 Global Battery Variables received from 681x commands
 These variables store the results from the LTC6813
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

/*************************************************************************
 Set configuration register. Refer to the data sheet
**************************************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {true,true,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {true,true,true,true}; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV=UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7]= {false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {false,true, false, true}; //!< Discharge time value //Dcto 1, 2, 4, 8  // Programmed for 30 minutes.   
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */

bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBITS[2]= {false,false}; //!< Digital Redundancy Path Selection//ps-0,1

// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_RREF   100e3
// significant digits
// voltages
#define LTCDEF_DIGITS_BATSLOW     2
#define LTCDEF_DIGITS_BATFAST     2
#define LTCDEF_DIGITS_BATFIFO_AVG 2
#define LTCDEF_DIGITS_CELL        3
// currents
#define LTCDEF_DIGITS_I1SLOW      2
#define LTCDEF_DIGITS_I2FAST      0
#define LTCDEF_DIGITS_I2FIFO_AVG  0
// power
#define LTCDEF_DIGITS_P1SLOW      2
// temperature
#define LTCDEF_DIGITS_TEMPSLOW    1
// set sense resistor here
#define LTCDEF_SENSE_RESISTOR 1e-4
#define LTCDEF_ERR_RETRIES 1
#define LTCDEF_POLL_EOC false
#define LTCDEF_CELLS_PER_CELL_MONITOR_COUNT 18

// chip select of the optional 2nd LTC6820
#define LTCDEF_GPO 9

// fast channel configuration: fast single shot, channel 2: I1, BAT (via P2 as voltage see below)
#define LTCDEF_FACTRL_CONFIG LTC2949_BM_FACTRL_FACH2
// ADC configuration (SLOT1 measures temperature via NTC, P2 measures voltage)
#define LTCDEF_ADCCFG_CONFIG (LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_P2ASV)

unsigned long mcuTime;
float AMS_Reading;
float StateOfCharge;
uint8_t retries = LTCDEF_ERR_RETRIES;
int faultArray[6] = {0, 0, 0, 0, 0, 0};
float TEMP_THREHOLD = 50;
float current;
float CURRENT_THRESHOLD = 10;
float OVER_VOLT_Thresh = 78.0;
float UNDER_VOLT_Thresh = 48.6;
float SOC;

#ifdef LTCDEF_LTC681X_ONLY
// polling not supported in case of LTCDEF_LTC681X_ONLY!!!
#define LTCDEF_POLL_EOC false
#else
// true for making ADCV command and poll EOC
// false for making ADCV command and wait until EOC
#define LTCDEF_POLL_EOC false
#endif

// define LTCDEF_IGNORE_PEC_ERRORS 
// quick work around for new 6815 style devices with PEC+CMDcnt 
// that is currently not supported by this source
#define LTCDEF_IGNORE_PEC_ERRORS
#undef LTCDEF_IGNORE_PEC_ERRORS
//
#ifdef LTCDEF_IGNORE_PEC_ERRORS
// ignore all PEC errors
inline bool err_detected(byte error) { return error > 0x3; }
// or just ignore single PEC errors
//inline bool err_detected(byte error) { return (ALLOW_ONE_PEC_ERROR ? ((error) > 1) : ((error) != 0)); }
#else
inline bool err_detected(byte error) { return error != 0; }
#endif

/*!**********************************************************************
 \brief  Initializes hardware and variables
  @return void
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock

  //Set up mode for pins 6 7 and 8
  //pinMode(Interlock_2_Pin, OUTPUT);
  pinMode(AMS_LED_PIN, OUTPUT);
  pinMode(Traction_LED_PIN, OUTPUT);
  pinMode(Interlock_Positive_Pin, OUTPUT);
  pinMode(Interlock_Negative_Pin, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);

  //Setup the communication with DC2350B.
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC,BMS_IC);

  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);   
  }   
  LTC6813_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC,BMS_IC);


  //Setup the DC2732 board communication
  LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);
  Init(LTCDEF__CS, false);
                                                                                      
}

/*!*********************************************************************
  \brief Main loop
   @return void
***********************************************************************/
void loop()
{
   if (Serial.available())           // Check for user input                      
  {                                                             
    uint32_t user_command;                                               
    user_command = read_int();      // Read the user command if == m keep looping
    if(user_command=='m')
    { }//Do nothing 
    else
    {run_command(user_command); 
    }
    user_command = user_command + 1;   
  }
}

/*!*****************************************
  \brief executes the user command
    @return void
*******************************************/
void run_command(uint32_t cmd)
{
  
  uint8_t streg=0; 
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read=0;
  
  switch (cmd)
  {
   
    case 1: 
      wakeup_sleep(TOTAL_IC);  
      LTC6813_wrcfg(TOTAL_IC,BMS_IC);
      LTC6813_wrcfgb(TOTAL_IC,BMS_IC);
      measurement_loop(DATALOG_ENABLED);
      break;
   
    case 2: // Enable a discharge transistor
      s_pin_read = select_s_pin();
      wakeup_sleep(TOTAL_IC);
      LTC6813_set_discharge(s_pin_read,TOTAL_IC,BMS_IC);
      LTC6813_wrcfg(TOTAL_IC,BMS_IC);
      LTC6813_wrcfgb(TOTAL_IC,BMS_IC);
      wakeup_idle(TOTAL_IC);
      error = LTC6813_rdcfg(TOTAL_IC,BMS_IC);
      error = LTC6813_rdcfgb(TOTAL_IC,BMS_IC);
      break;
    
    case 3: // Clear all discharge transistors

      wakeup_sleep(TOTAL_IC);
      LTC6813_clear_discharge(TOTAL_IC,BMS_IC);
      LTC6813_wrcfg(TOTAL_IC,BMS_IC);
      LTC6813_wrcfgb(TOTAL_IC,BMS_IC);
      wakeup_idle(TOTAL_IC);
      error = LTC6813_rdcfg(TOTAL_IC,BMS_IC);
      error = LTC6813_rdcfgb(TOTAL_IC,BMS_IC);
      Serial.print("9");
      Serial.println();
      break;
               
    case 'm': //prints menu
    
      break;

    default:
      break;
  }
}

/*!**********************************************************************************************************************************************
 \brief For writing/reading configuration data or measuring cell voltages or reading aux register or reading status register in a continuous loop  
 @return void
*************************************************************************************************************************************************/
void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  char input = 0;
  
  while (input != 'm')
  {
     if (Serial.available() > 0)
      {
        input = read_char();
      } 
      //Read Voltage on Analog pin #1
      //If the analog reading is higher than 2.3, then turn pin 7 and 8 high
      //If not make sure they are low- maybe we want to reduce this.
      AMS_Reading  = analogRead(AMS_SWITCH)/1024.0*5;
      if (AMS_Reading  >=  2.0 && faultArray[3] != 1 && faultArray[4] != 1){
        
        digitalWrite(Interlock_Positive_Pin, HIGH);
        digitalWrite(Interlock_Negative_Pin, HIGH);
        digitalWrite(AMS_LED_PIN, HIGH);
      }else{
        digitalWrite(Interlock_Positive_Pin, LOW);
        digitalWrite(Interlock_Negative_Pin, LOW);
        digitalWrite(AMS_LED_PIN, LOW);
      }

  
      if (WRITE_CONFIG == ENABLED)
      {
        wakeup_idle(TOTAL_IC);
        LTC6813_wrcfg(TOTAL_IC,BMS_IC);
        LTC6813_wrcfgb(TOTAL_IC,BMS_IC);

      }
    
      if (READ_CONFIG == ENABLED)
      {
        wakeup_idle(TOTAL_IC);
        error = LTC6813_rdcfg(TOTAL_IC,BMS_IC);
        error = LTC6813_rdcfgb(TOTAL_IC,BMS_IC); 
        print_rxconfig();
      }
    
      if (MEASURE_CELL == ENABLED)
      {
        wakeup_idle(TOTAL_IC);
        LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
        LTC6813_pollAdc();
        wakeup_idle(TOTAL_IC);
        error = LTC6813_rdcv(0, TOTAL_IC,BMS_IC);
 
        print_cells(datalog_en);
      }
      
      if (MEASURE_STAT == ENABLED)
      {
        wakeup_idle(TOTAL_IC);
        LTC6813_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);                                    ///\\\Send STAT valuyes before AUX 
        LTC6813_pollAdc();
        wakeup_idle(TOTAL_IC);
        error = LTC6813_rdstat(0,TOTAL_IC,BMS_IC); // Set to read back all aux registers
      
        print_stat();
      }  
        
      if (MEASURE_AUX == ENABLED)
      {
        wakeup_idle(TOTAL_IC);
        LTC6813_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
        LTC6813_pollAdc();
        wakeup_idle(TOTAL_IC);
        error = LTC6813_rdaux(0,TOTAL_IC,BMS_IC); // Set to read back all aux registers
    
        print_aux(datalog_en);
      }



      //Prints the temperature Values
      getTemperature();

      //Get values from DC2732
      
      if (DC2732 == ENABLED){
         DC2732_Loop();  
      }



      
      //Faults 
      fault_detection();


      //Open wire loop. 
      
//      if (OPEN_WIRE_LOOP == DISABLED)
//      {
//              LTC6813_run_openwire_multi(TOTAL_IC, BMS_IC);
//      }
      
      Serial.println();    //Prints new line. only place to do this. 
      
      delay(MEASUREMENT_LOOP_TIME);
  } 
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
  @return void
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 1)
    {
      
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++)
      {
        if (i == 10 || i == 12){
            float Bank_Voltage = BMS_IC[current_ic].cells.c_codes[i]*0.0001 - 0.3;
            Serial.print(Bank_Voltage,4);}
        else if (i == 11){
            float Bank_Voltage1 = BMS_IC[current_ic].cells.c_codes[i]*0.0001 + 0.6;
            Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001 + 0.6,4);}
        else{
            float Bank_Voltage2 = BMS_IC[current_ic].cells.c_codes[i]*0.0001;
            Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        }
        Serial.print(",");
      }
    }
  }
}


/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void fault_detection(){
 
  //faultArray[];FAULT_LED

 for (int i = 0; i < 7; i++){
  
  Serial.print(faultArray[i]); 
  Serial.print(",");

  //Short Circuit
  if (AMS_Reading >= 0 && AMS_Reading < 0.5 && StateOfCharge >= 2 && StateOfCharge < 4.5){
    faultArray[0] = 1;
  }

  //Open Circuit
  if (AMS_Reading >= 2 && StateOfCharge >= 0 && StateOfCharge < 0.5){
    faultArray[1] = 1;
  }

  //over current
  if (current > CURRENT_THRESHOLD){
    faultArray[3] = 1;    
  }

  //Over Voltage
  if (SOC > OVER_VOLT_Thresh){
    faultArray[5] = 1;
  }else if (SOC < UNDER_VOLT_Thresh){
    faultArray[4] = 1;    
  }
  
  if (faultArray[i] == 1){
      digitalWrite(FAULT_LED, HIGH);
  }
  
 }
}
/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 1)
      {
        
      Serial.print(BMS_IC[current_ic].aux.a_codes[5]*0.0001,4); //V reference
      Serial.print(",");
      
      for (int i=6; i < 8; i++)
      {
        float voltage = BMS_IC[current_ic].aux.a_codes[i]*0.0001;
        float ntcResistance = voltage*50000.0/(3.0-voltage);
        float temperature = 1.0/(float)(A_1 + B_1*log(ntcResistance)+C_1*(pow(log(ntcResistance), 3))) - 273.15; 
           
        if (ntcResistance < 0){
          temperature = 0.0;
        }
        Serial.print(temperature);  
        Serial.print(",");                                                                        
      }
     }
  }
}


/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_stat(void)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    double itmp;
    SOC = BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30;
    Serial.print(SOC,4);             ///State of Charge (Commulative Voltage)
    Serial.print(F(","));
    itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0076)) - 276);   //Internal Die Temperature(°C) = ITMP • (100 µV / 7.6mV)°C - 276°C
    Serial.print(itmp,4);
    Serial.print(F(","));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2]*0.0001,4);                ///Regulator A
    Serial.print(F(","));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[3]*0.0001,4);                ///Regulator D
    Serial.print(F(","));
  }
}


/*!****************************************************
  \brief Function to select the S pin for discharge
  @return void
 ******************************************************/
int8_t select_s_pin(void)
{
  int8_t read_s_pin=0;
  read_s_pin = (int8_t)read_int();
  return(read_s_pin);
}

/*!****************************************************************************
  \brief prints data which is written on COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrcomm(void)
{
 int comm_pec;

  Serial.println(F("Written Data in COMM Register: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic+1,DEC);
    
    for(int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6,&BMS_IC[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec)); 
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxcomm(void)
{
   Serial.println(F("Received Data in COMM register:"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));                                                                       
    Serial.print(current_ic+1,DEC);
    
    for(int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
   \brief Function to print in HEX form
   @return void
 *****************************************************************************/
void serial_print_hex(uint8_t data){
  if (data< 16)
  {
    Serial.print("0");
    Serial.print((byte)data,HEX);
  }
  else
    Serial.print((byte)data,HEX);
}


void getTemperature(){

    for ( int i = 0; i < 16; ++i){

       //Make sure to turn off the 0x4D board when using the 0x4C
       if (i == 0){
          Wire.beginTransmission(0x4C);   
          Wire.write(byte(0b00000000));
          Wire.endTransmission();         
       }
       //Make sure to turn off the 0x4C board when using the 0x4D
       if (i == 8){
          Wire.beginTransmission(0x4D);  
          Wire.write(byte(0b00000000));
          Wire.endTransmission();          
       }


       
       if (i < 8){
          Wire.beginTransmission(0x4D);  //Begin Communication //0x4C: 1001100 0x4D: 1001101   
          Wire.write(pins[i]);
          Wire.endTransmission(); 
        
          float thermistorReading = (float) analogRead(THERMISTOR_PIN)/1023.0*5.0; //read from analog pin A0
          float ntcResistance = (3.3-thermistorReading)*100000/(thermistorReading);
          float temperature = 1.0/(float)(A + B*log(ntcResistance)+C*(pow(log(ntcResistance), 3))) - 273.15;

          if (temperature > TEMP_THREHOLD){
             faultArray[2] = 1;
          }
          
          Serial.print(temperature);                  
          Serial.print(",");
       } 
       if (i >= 8 && i < 16){        
          Wire.beginTransmission(0x4C);  //Begin Communication //0x4C: 1001100 0x4D: 1001101   
          Wire.write(pins[15-i]);
          Wire.endTransmission(); 
             
          float thermistorReading = (float) analogRead(A0)/1023.0*5.0; //read from analog pin A0
          float ntcResistance = (3.3-thermistorReading)*100000/(thermistorReading);
          float temperature = 1.0/(float)(A + B*log(ntcResistance)+C*(pow(log(ntcResistance), 3))) - 273.15;
   
          if (temperature > TEMP_THREHOLD){
             faultArray[2] = 1;
          }
          
          Serial.print(temperature);                  
          //Do not print comma at the end. 
         // if (i < 15){
          Serial.print(",");
        //  }          
       } 
       
       delay(10); // give it abit of delay to read. 

     } 
}



void Init(uint8_t selCS, boolean ltc2949onTopOfDaisychain)
{
#ifdef LTCDEF_LTC681X_ONLY
  ltc2949onTopOfDaisychain = true;
#endif

  LTC2949_CS = selCS;
  //Initialize LTC2949 library
  LTC2949_init_lib(LTCDEF_CELL_MONITOR_COUNT, ltc2949onTopOfDaisychain,
    /*boolean debugEnable       */false);
   
  LTC2949_init_device_state();

  delay(LTC2949_TIMING_BOOTUP);
 
  byte error = 0;
#ifdef LTCDEF_LTC681X_ONLY
  mcuTime = millis();
  error |= CellMonitorInit();
#else
  error = WakeUpReportStatus();
  error |= Cont(false);
  mcuTime = millis();
  delay(LTC2949_TIMING_CONT_CYCLE);
  error |= CellMonitorInit();
  error |= Cont(true);
#endif

  //Do nothing

}

void DC2732_Loop(){
  
  String str = "";
  
#ifndef LTCDEF_LTC681X_ONLY
  int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
#endif

  uint16_t cellMonDat[LTCDEF_CELL_MONITOR_COUNT * 3]; // = LTCDEF_CELL_MONITOR_COUNT * 6 bytes
  byte * buffer = (byte *)cellMonDat; // buffer and cellMonDat can share the same memory
  byte  error = 0;
  unsigned long timeBuffer = millis();
  unsigned long timerus = micros();

#ifndef LTCDEF_LTC681X_ONLY
  // store the last TBx value that was read (to be able to calc time difference later...)
  uint32_t deltaT = LTC2949_GetLastTBxInt();
#endif

#ifdef LTCDEF_DO_RESET_TEST
  if (deltaT > LTCDEF_DO_RESET_TEST / 1.0e3 / LTC2949_LSB_TB1)    //no need\\
  {
    LTC2949_reset();
    delay(LTC2949_TIMING_AUTO_SLEEP_MAX*1.5);
    Init(LTC2949_CS, LTC2949_onTopOfDaisychain);
    return;
  }
#endif // LTCDEF_DO_RESET_TEST

  // LTC2949_ChkUpdate checks for changed in TBx (default is to check TB4)
  // this tells if we have updated values available in the slow channel registers
#ifdef LTCDEF_LTC681X_ONLY
  const boolean slowChannelReady = false;
#else
  boolean slowChannelReady = LTC2949_ChkUpdate(&error);
  if (slowChannelReady || LTC_TIMEOUT_CHECK(timeBuffer, mcuTime + LTC2949_TIMING_CONT_CYCLE))
  {
    // in case of any error below we will also enter here! (see last delay(LTC2949_TIMING_IDLE2CONT2UPDATE))
    error |= ChkDeviceStatCfg();
    slowChannelReady = true;
    
  }
#endif


 
#ifndef LTCDEF_LTC681X_ONLY


  //CURRENT\\ 
  if (slowChannelReady)
  {
    error |= LTC2949_READ(LTC2949_VAL_I1, 3, buffer);
    current = LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR;
    str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW); //Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW);
  }

 
  str += ',';//PrintComma();

  //POWER\\ 
  if (slowChannelReady)
  {
    error |= LTC2949_READ(LTC2949_VAL_P1, 3, buffer);
    float numberOfBanks = 17.7;
    str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR * numberOfBanks, LTCDEF_DIGITS_P1SLOW); //Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_P1SLOW);
  }
  str += ',';//PrintComma();

  //BAT
  if (slowChannelReady)
  {
    error |= LTC2949_READ(LTC2949_VAL_BAT, 2, buffer);
    float BAT_Voltage = LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT ;
    StateOfCharge = BAT_Voltage;
    str += String(BAT_Voltage, LTCDEF_DIGITS_BATSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, LTCDEF_DIGITS_BATSLOW);
   
    if (BAT_Voltage > 2.6 && BAT_Voltage <= 5.0){
      digitalWrite(Traction_LED_PIN, HIGH);
    }else{
     digitalWrite(Traction_LED_PIN, LOW);
    }
  
  }
  str += ',';//PrintComma();

  //CHIP Temperature
  if (slowChannelReady)
  {
    error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, buffer);
    str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
  }
  str += ',';//PrintComma();

  //Internal Temperature
  if (slowChannelReady)
  {
    error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, buffer);
    str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
  }
  str += ',';//PrintComma();
#endif


  ////////////////////////////////////////////////////////////////////
  // fast synchronous cell voltage and current measurement
  ////////////////////////////////////////////////////////////////////
  // clear old cell voltage conversion results
  error |= LTC2949_68XX_ClrCells();
  
//  error |= CellMonitorCFGA((byte*)cellMonDat, false);

  if (err_detected(error))
  {
    str = "";

    delay(100); // we wait 0.1 second, just avoid too many trials in case of error.

    if (retries--)
    {
      Init(LTC2949_CS, LTC2949_onTopOfDaisychain);
    }
    else
    {
      // lets try to toggle isoSPI bus configuration and LTC6820 master (four possible combinations)
      // Note: here we treat all possible combinations to be equally likely. This way the boards
      // can be connected together in any way.
      // In a real system only two combinations would make sense.
      switch (LTC2949_CS + (LTC2949_onTopOfDaisychain ? (uint8_t)128 : (uint8_t)0))
      {
      case (LTCDEF__CS + 0):
        Init(LTCDEF_GPO, false);
        break;
      case (LTCDEF_GPO + 0):
        Init(LTCDEF__CS, true);
        break;
      case (LTCDEF__CS + 128):
        Init(LTCDEF_GPO, true);
        break;
      case (LTCDEF_GPO + 128):
      default:
        Init(LTCDEF__CS, false);
        break;
      }
      retries = LTCDEF_ERR_RETRIES;
    }
    return;
  }


  // trigger measurement (broadcast command will trigger cell voltage and current measurement)
  error |= LTC2949_ADxx(MD_FAST,CELL_CH_ALL,DCP_DISABLED,
    LTCDEF_POLL_EOC ? LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_68XX_T6C_27KHZ_US) : 0
  );



  cellMonDat[0] = 0xFFFFU; // will be used later

#ifdef LTCDEF_LTC681X_ONLY
#else                       
  if (!LTCDEF_POLL_EOC)
  {
    timeBuffer = micros();
    // for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
    while (false && !LTC_TIMEOUT_CHECK(micros(), timeBuffer + LTC2949_68XX_T6C_27KHZ_US))
      ; // wait for all cell voltage measurements to be completed.
  }
  fastData2949[LTC2949_RDFASTDATA_HS] = 0; // clear the HS bytes
  // poll LTC2949 for conversion done
  error |= LTC2949_RdFastData(
    fastData2949,
    cellMonDat,
    LTC2949_68XX_CMD_RDCVA,
    LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_FASTSSHT_RDY_TIME_US));
  // calc cycle time (ADCV to "results ready")
  deltaT = micros() - timeBuffer;
  // check if we already read valid data from LTC2949
  if (LTC2949_FASTSSHT_HS_OK(fastData2949))
  {
    ; // all fine, nothing to do.
  }
  else if (LTC2949_FASTSSHT_HS_CLR(fastData2949))
  {
    // we polled for HS==0x0F before, so it cannot be all HS are zero! something went wrong (e.g. timeout)
    error |= LTC2949_ERRCODE_OTHER;
  }
  else if (LTC2949_FASTSSHT_HS_LAST_OK(fastData2949)) // first HS != 0x0F, last HS == 0x0F
  {
    // we have to read data from LTC2949 again, as only the next RDCVx will report the final conversion results
    // also cell voltages have to be read again, as also those most probably were not updated
    // note: here we must not poll HS! (it must be zero now!)
    error |= LTC2949_RdFastData(
      fastData2949,
      cellMonDat,
      LTC2949_68XX_CMD_RDCVA);

    if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // HS must be cleared now
      error |= LTC2949_ERRCODE_OTHER; // this must never happen in case of fast single shot events
  }
  else
  {
    // Unexpected HS bytes, something went wrong
    error |= LTC2949_ERRCODE_OTHER;
  }
#endif



#ifndef LTCDEF_LTC681X_ONLY
  // print fast I2
  str += String(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I2FAST); //Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I2FAST);
  str += ',';//PrintComma();
  // print fast BAT
  str += String(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST); //Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST);
  // clear the EOC by reading again if necessary:
  if (!LTC2949_FASTSSHT_HS_CLR(fastData2949))
    error |= LTC2949_RdFastData(fastData2949);
  if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // for sure HS bytes must be cleared now
    error |= LTC2949_ERRCODE_OTHER;

  str += ',';//PrintComma();
#endif

  Serial.print(str);
  str = "";


  
}


/*!*********************************************************************
\brief Wakeup LTC2949, report and clear all status / alert registers
***********************************************************************/
byte WakeUpReportStatus()
{
  byte  error = LTC2949_WakeupAndAck();
  error |= LTC2949_ReadChkStatusFaults(true, true);
  return error;
}


byte CellMonitorInit()
{
  byte cellMonDat[LTCDEF_CELL_MONITOR_COUNT * 6];
  LTC2949_68XX_RdCfg(cellMonDat); // dummy read
  // dummy read of cell voltage group A
  byte error = ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
  // clear all cell voltage groups
  error |= LTC2949_68XX_ClrCells();

  error |= CellMonitorCFGA(cellMonDat, true);

  // read configuration and print
  error |= LTC2949_68XX_RdCfg(cellMonDat);
  SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
  PrintComma();
  error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
  // trigger cell voltage measurement of all cells in fast mode
  error |= LTC2949_ADxx(MD_FAST,CELL_CH_ALL,DCP_DISABLED,0);
  delay(2); // wait for conversion results ready to be read
  // print all conversion results
  error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
  return error;
}



byte Cont(boolean enable)
{
  if (enable)
  {
    byte error = 0;
#ifdef LTCDEF_READ_FROM_EEPROM
    error = LTC2949_EEPROMRead();
#else
    // fast slot not used, still we configure something
    LTC2949_SlotFastCfg(3, 2);
    // SLOT1 measures temperature via NTC between V1 and GND. SLOT2 not used, still we configure something
    LTC2949_SlotsCfg(1, 0, 4, 5);
    // enable NTC temperature measurement via SLOT1
    NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);
#endif

#ifdef LTCDEF_WRITE_TO_EEPROM
    error = LTC2949_EEPROMWrite();
#endif

    // read & clear status
    error |= LTC2949_ReadChkStatusFaults(true, false);

    // enable measurement
    return error | LTC2949_GoCont(
      /*cfgFast:      */ LTCDEF_FACTRL_CONFIG,
      /*byte adcCfg:  */ LTCDEF_ADCCFG_CONFIG);
  }
  LTC2949_WriteFastCfg(0);
  LTC2949_OpctlIdle();
  return 0;
}


byte ReadPrintCellVoltages(uint16_t rdcv, uint16_t * cellMonDat)
{
  // in case LTC2949 is parallel to the daisychain, we now read only the cell voltages
  byte error = LTC2949_68XX_RdCells(rdcv, cellMonDat);
  PrintCellVoltages(cellMonDat, true);
  return error;
}


byte CellMonitorCFGA(byte * cellMonDat, bool verbose)
{
  // read configuration and print
  byte error = LTC2949_68XX_RdCfg(cellMonDat);
  if (verbose)
  {
    SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
    PrintComma();
  }
  // set REFON in configuration registers
  for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
  {
    cellMonDat[i * 6 + 0] = 1 << 2; //REFON, all others zero
    cellMonDat[i * 6 + 4] = 0; //clear all DCC
    cellMonDat[i * 6 + 5] = 0; //clear all DCC
  }
  // write configuration registers
  error |= LTC2949_68XX_WrCfg(cellMonDat);
  return error;
}



void NtcCfgWrite(int ntc1or2, float rref, float a, float b, float c)
{
  byte data[3];
  LTC2949_FloatToF24Bytes(rref, data);
  LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_RREF2 : LTC2949_VAL_RREF1, 3, data);
  LTC2949_FloatToF24Bytes(a, data);
  LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2A : LTC2949_VAL_NTC1A, 3, data);
  LTC2949_FloatToF24Bytes(b, data);
  LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2B : LTC2949_VAL_NTC1B, 3, data);
  LTC2949_FloatToF24Bytes(c, data);
  LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2C : LTC2949_VAL_NTC1C, 3, data);
}


void PrintCellVoltages(uint16_t * cellMonDat, boolean init)
{
  for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT * 3; i++)
  {
    if (cellMonDat[i] != 0xFFFFU) // don't print non-existing cells
      Serial.print(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
    if (init)
      cellMonDat[i] = 0xFFFFU; // this allows to check if next cell voltages were read correctly
    PrintComma();
  }
}


byte ChkDeviceStatCfg()
{
  byte error;
  byte data[10];
  byte dataOthers;
  boolean expChkFailed;

  // check STATUS (EXT)FAULTS, ALERT registers
  error = LTC2949_ReadChkStatusFaults(
    /*boolean lockMemAndClr:    */ false,
    /*boolean printResult:      */ false,
    /*byte len:                 */ 10,
    /*byte * statFaultsExpAndRd:*/ data,
    /*boolean * expChkFailed:   */ &expChkFailed,
    /*byte expDefaultSet):      */ LTC2949_STATFAULTSCHK_IGNORE_STATUPD | LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR);

  if (err_detected(error))
    return error;

  if (expChkFailed)
  {
    error = LTC2949_ERRCODE_OTHER; // STATUS (EXT)FAULTS, ALERT check failed
    SerialPrintByteArrayHex(data, 10, true); // report the status
    PrintComma();
    return LTC2949_ERRCODE_OTHER;
  }

  // check BRCEN bit
  if (err_detected(error = LTC2949_READ(LTC2949_REG_REGSCTRL, 1, &dataOthers)))
    return error; // PEC error
  if (bitMaskSetClrChk(dataOthers, LTC2949_BM_REGSCTRL_BCREN, !LTC2949_onTopOfDaisychain))
    return LTC2949_ERRCODE_OTHER; // BRCEN != LTC2949_onTopOfDaisychain

  if (err_detected(error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &dataOthers)))
    return error; // PEC error
  if (dataOthers != LTC2949_BM_OPCTRL_CONT)
    return LTC2949_ERRCODE_OTHER; // not in continuous mode

  if (err_detected(error = LTC2949_READ(LTC2949_REG_FACTRL, 1, &dataOthers)))
    return error; // PEC error
  if (dataOthers != LTCDEF_FACTRL_CONFIG)
    return LTC2949_ERRCODE_OTHER;  // not or wrong fast mode

  if (err_detected(error = LTC2949_ADCConfigRead(&dataOthers)))
    return error; // PEC error
  if (dataOthers != LTCDEF_ADCCFG_CONFIG)
    return LTC2949_ERRCODE_OTHER; // wrong ADC configuration

  return 0;
}


String CellVoltagesToString(uint16_t * cellMonDat, uint8_t nic)
{
  String str = "";
  uint8_t i = nic * 3;
  uint8_t lasti = i + 3;
  for (; i < lasti; i++)
  {
    if (cellMonDat[i] != 0xFFFFU)
      str += String(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
    str += ',';
  }
  return str;
}


/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6813 to the serial port.
 @return void
 *******************************************************************/
void print_rxconfig(void)
{
  Serial.println(F("Received Configuration A Register: "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    Serial.println("\n");
  }
}
