// Import Libraries\\
import processing.javafx.*;
import java.awt.Frame;
import java.awt.BorderLayout;
import controlP5.*; 
import processing.serial.*;
import processing.sound.*;

// Create objects from available libraries \\
Serial serialPort;  
SoundFile Can_Not_Connect;
SoundFile ConnectSound;
PFont DigitalFont;
PFont LatoFont;
PFont Default;
PImage OpenWireImage;
PrintWriter SerialOutput;
PImage needle; // to store meter needle image


// Intial Location for Mointoring Board widget 
int x_begin_loc = 50;
int y_begin_loc = 80;

// Intial Location for Legends widget 
int x_begin_loc1 = 50;
int y_begin_loc1 = 790;

// Intial Location for widget Status widget 
int x_begin_loc2 = 675;
int y_begin_loc2 = 80;

// Intial Location for Data widget 
int x_begin_loc3 = 675;
int y_begin_loc3 = 80;

// Intial Location for Discharge control Status 
int x_begin_loc4 = 1285;
int y_begin_loc4 = 30;

// Intial Location for DC2350B disgonstic
int x_begin_loc5 = 475;
int y_begin_loc5 = 790;

// Intial Location for DC2732 disgonstic
int x_begin_loc6 = 840;
int y_begin_loc6 = 790;

// Intial Location for Console Window
int x_begin_loc7 = 1235;
int y_begin_loc7 = 790;

// Intial Location for indicator window
int x_begin_loc8 = 1020;
int y_begin_loc8 = 490;

// Battery System Variables 
int BAUD_RATE = 115200;
String PORT = "    --";
float SOC;
float CURRENT;
float PercentageOfSOC;
float CHIP_TEMP;  //DC2350B
float CHIP_TEMP1; //DC2732
float MAXIMUM_TEMP = 0;
float POWER;
float ENERGY;
String bufferIN;
float RegA;
float RegB;
float Vref;
float VBAT;
float FAST_CURRENT;
float FAST_BAT;
float Internal_TEMP; 
float Maximum_Cell_Difference = 0;
float MinimumCellVotlage = 0;
float MaximumCellVoltage = 0;
float bank_voltages[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float bank_temperature_array[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
boolean Dicharge_Status[] =  {false, false, false, false,false, false, false, false, false, false, false, false,false, false, false, false, false, false};
boolean OPEN_WIRE_ARRAY[] = {false, false, false, false,false, false, false, false, false, false, false, false,false, false, false, false, false, false};
boolean OverDischargeButtonsArray[] = {false, false, false, false,false, false, false, false, false, false, false, false,false, false, false, false, false, false};
boolean SET_DISCHARGE[] =  {false, false, false, false,false, false, false, false, false, false, false, false,false, false, false, false, false, false};
color bank_color_array[] = {color(255), color(255), color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255),color(255)};
int fault_Status[] = {0, 0, 0, 0, 0, 0, 0};


// System Threholds
float TEMP_THRESHOLD = 35;
float MAX_VOLTAGE = 4.3;                          
float MINIMUM_VOLTAGE = 2.7;
float MAX_SOC_VOLTAGE = 75.6;
float MAX_CHIP_TEMP = 35;
float MIN_CHIP_TEMP = 10;
float MIN_SOC_VOLTAGE = MINIMUM_VOLTAGE*18;
float MAX_CURRENT = 10.0;
float MIN_CURRENT = 0.0;
float MAX_ENERGY = 189;
float MIN_ENERGY = MINIMUM_VOLTAGE*18*2.5;
float MAX_POWER = 20*4.2*18;
float MIN_POWER = 3.2*18;
//Logic Variables
boolean StartReading = false;
boolean overDC2350B_Option = false;
boolean overDC2732_Option = false;
boolean DC2732 = false;
boolean DC2350B = true;
boolean overConnectButton = false;
boolean overClearDischargeButton = false;
boolean Try_TO_CONNECT = true;
boolean CONNECT_Disconnect = false;
boolean CLEAR_DISCHARGE = false;
boolean Intialized = false;
String CONNECTION_STATUS = " Not Connected";
String connect_DisConnect_Button = "  Connect";
String LoggingStatusString = "     Not Saving";
String myString;
String myString1;
String[] myStringArray;
String[] SerialValues;
int clickSoundCount = 0;
int clickSoundCount1 = 0;
float ProgressIndicator;
color Connection_Status_Color = color(255,0,0);


boolean increment= false;
boolean increment1 = false;

void settings(){
  
  size(1535, 960, FX2D);
  //fullScreen(FX2D);

}

void setup() {
  
  //App Icon, Title, Font and BG Color
  PImage icon = loadImage("logo.PNG");
  PImage TrueLogo = loadImage("TrueLogo.PNG");
  needle=loadImage("ndl.png"); //loading ndl.png image to needle

  OpenWireImage = loadImage("Open.png");
  DigitalFont = createFont("digital.ttf", 13);
  LatoFont = createFont("number.ttf", 13);
  Default = createFont("Arial", 12);
  SerialOutput = createWriter("data_DC2350.txt");
  //textFont(LatoFont);
  Can_Not_Connect = new SoundFile(this, "disconnect.wav");
  ConnectSound = new SoundFile(this, "connect.mp3");
  surface.setTitle("TruBalance GUI");
  surface.setIcon(icon);
  surface.setResizable(false);
  background(color(#F0F0F0));
  //textFont(Default);
  
  IntializeSerial();

  image(TrueLogo, 1025, 725, 500, 250);
  
  //GUI
  fill(0);
  text("MOINTORING BOARD", x_begin_loc + 25, y_begin_loc + 5);
  text("CHARGE CONTROL", x_begin_loc4 + 25, y_begin_loc4 + 55);
 // text("INDICATORS", x_begin_loc8 + 25, y_begin_loc8 + 55);
  text("CONNECTION", x_begin_loc + 25, y_begin_loc -55);
  text("DATA", x_begin_loc3 + 25, y_begin_loc3 -55);
  text("Status: ", x_begin_loc3 + 25, y_begin_loc3 -33);
  text("Time: ", x_begin_loc3 + 200, y_begin_loc3 -33);
  text("Connection Status: ", x_begin_loc + 25, y_begin_loc -30);
  text("Port: ", x_begin_loc + 240, y_begin_loc -30);
  text("Buad Rate: ", x_begin_loc + 320, y_begin_loc -30);
  text("LEGENDS", x_begin_loc1 + 25, y_begin_loc1 + 5);
  text("DC2350B Diagonsis", x_begin_loc5 + 25, y_begin_loc5 + 5);
  text("DC2732 Diagonsis", x_begin_loc6 + 25, y_begin_loc6 + 5); 
 // text("Console Window", x_begin_loc7 + 25, y_begin_loc7 + 5); 
  text("2350", x_begin_loc + 460, y_begin_loc - 25); 
  text("2732", x_begin_loc + 460, y_begin_loc - 40); 
  text("BATTERY STATUS", x_begin_loc2 + 25, y_begin_loc2 + 5);
  

  stroke(180);
  
  //Widget for Connection.
  line(x_begin_loc, y_begin_loc-60, x_begin_loc + 20, y_begin_loc-60);
  line(x_begin_loc + 105, y_begin_loc-60, x_begin_loc + 600, y_begin_loc-60);
  line(x_begin_loc, y_begin_loc-60, x_begin_loc, y_begin_loc-20);
  line(x_begin_loc, y_begin_loc-20, x_begin_loc + 600, y_begin_loc-20);
  line(x_begin_loc + 600, y_begin_loc-20, x_begin_loc + 600, y_begin_loc-60);
  
  
  //Widget for Mointoring Board.
  line(x_begin_loc, y_begin_loc, x_begin_loc + 20, y_begin_loc);
  line(x_begin_loc + 150, y_begin_loc, x_begin_loc + 600, y_begin_loc);
  line(x_begin_loc, y_begin_loc, x_begin_loc, y_begin_loc + 685);
  line(x_begin_loc, y_begin_loc + 685, x_begin_loc + 600, y_begin_loc + 685);
  line(x_begin_loc + 600, y_begin_loc + 685, x_begin_loc + 600, y_begin_loc);
  
  //Bank Widget
  BatteryBankFigure(x_begin_loc + 20, y_begin_loc + 20, "Bank 1");
  BatteryBankFigure(x_begin_loc + 20, y_begin_loc + 130, "Bank 2");
  BatteryBankFigure(x_begin_loc + 20, y_begin_loc + 240, "Bank 3");
  BatteryBankFigure(x_begin_loc + 20, y_begin_loc + 350, "Bank 4");
  BatteryBankFigure(x_begin_loc + 20, y_begin_loc + 460, "Bank 5");
  BatteryBankFigure(x_begin_loc + 20, y_begin_loc + 570, "Bank 6");

  BatteryBankFigure(x_begin_loc + 210, y_begin_loc + 20, "Bank 7");
  BatteryBankFigure(x_begin_loc + 210, y_begin_loc + 130, "Bank 8");
  BatteryBankFigure(x_begin_loc + 210, y_begin_loc + 240, "Bank 9");
  BatteryBankFigure(x_begin_loc + 210, y_begin_loc + 350, "Bank 10");
  BatteryBankFigure(x_begin_loc + 210, y_begin_loc + 460, "Bank 11");
  BatteryBankFigure(x_begin_loc + 210, y_begin_loc + 570, "Bank 12");
  
  BatteryBankFigure(x_begin_loc + 410, y_begin_loc + 20, "Bank 13");    
  BatteryBankFigure(x_begin_loc + 410, y_begin_loc + 130, "Bank 14");    
  BatteryBankFigure(x_begin_loc + 410, y_begin_loc + 240, "Bank 15");   
  BatteryBankFigure(x_begin_loc + 410, y_begin_loc + 350, "Bank 16");   
  BatteryBankFigure(x_begin_loc + 410, y_begin_loc + 460, "Bank 17");   
  BatteryBankFigure(x_begin_loc + 410, y_begin_loc + 570, "Bank 18");
  
  //Batteries in Bank Widget.
  batteryFigure(x_begin_loc + 30, y_begin_loc + 53, 60, 60);
  batteryFigure(x_begin_loc + 30, y_begin_loc + 163, 60, 60);
  batteryFigure(x_begin_loc + 30, y_begin_loc + 273, 60, 60);
  batteryFigure(x_begin_loc + 30, y_begin_loc + 383, 60, 60);
  batteryFigure(x_begin_loc + 30, y_begin_loc + 493, 60, 60);
  batteryFigure(x_begin_loc + 30, y_begin_loc + 603, 60, 60);
  //Column #2
  batteryFigure(x_begin_loc + 220, y_begin_loc + 53, 60, 60);
  batteryFigure(x_begin_loc + 220, y_begin_loc + 163, 60, 60);
  batteryFigure(x_begin_loc + 220, y_begin_loc + 273, 60, 60);
  batteryFigure(x_begin_loc + 220, y_begin_loc + 383, 60, 60);
  batteryFigure(x_begin_loc + 220, y_begin_loc + 493, 60, 60);
  batteryFigure(x_begin_loc + 220, y_begin_loc + 603, 60, 60);
  //Column #3
  batteryFigure(x_begin_loc + 420, y_begin_loc + 53, 60, 60);
  batteryFigure(x_begin_loc + 420, y_begin_loc + 163, 60, 60);
  batteryFigure(x_begin_loc + 420, y_begin_loc + 273, 60, 60);
  batteryFigure(x_begin_loc + 420, y_begin_loc + 383, 60, 60);
  batteryFigure(x_begin_loc + 420, y_begin_loc + 493, 60, 60);
  batteryFigure(x_begin_loc + 420, y_begin_loc + 603, 60, 60);
  
  //Charge Control Status
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 140, "Bank 1");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 170, "Bank 2");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 200, "Bank 3");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 230, "Bank 4");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 260, "Bank 5");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 290, "Bank 6");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 320, "Bank 7");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 350, "Bank 8");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 380, "Bank 9");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 410, "Bank 10");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 440, "Bank 11");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 470, "Bank 12");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 500, "Bank 13");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 530, "Bank 14");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 560, "Bank 15");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 590, "Bank 16");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 620, "Bank 17");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 650, "Bank 18");
  BatteryControlBox(x_begin_loc4 + 15, y_begin_loc4 + 680, "Max Diff");
  
  text("STATUS", x_begin_loc4 + 80, y_begin_loc4 + 130);
  text("MANUAL", x_begin_loc4 + 140, y_begin_loc4 + 130);
  
  //Widget for Legend.
  stroke(180);
  strokeWeight(1);
  line(x_begin_loc1, y_begin_loc1, x_begin_loc1 + 20, y_begin_loc1);
  line(x_begin_loc1 + 80, y_begin_loc1, x_begin_loc1 + 410, y_begin_loc1);
  line(x_begin_loc1, y_begin_loc1, x_begin_loc1, y_begin_loc1 + 160);
  line(x_begin_loc1, y_begin_loc1 + 160, x_begin_loc1 + 410, y_begin_loc1 + 160);
  line(x_begin_loc1 + 410, y_begin_loc1 + 160, x_begin_loc1 + 410, y_begin_loc1);  
 
  //Battery figures in Legend
  BatteryLegendFigures(x_begin_loc1+50, y_begin_loc1+70, 40, 40,color(255, 50, 0), "  HOT", "OverTempThresh");
  BatteryLegendFigures(x_begin_loc1+140, y_begin_loc1+70, 40, 40,color(255, 192, 192), "DANGER", "    OverVoltage");
  BatteryLegendFigures(x_begin_loc1+230, y_begin_loc1+70, 40, 40,color(0, 255, 0), "GOOD", "  VoltageInRange");
  BatteryLegendFigures(x_begin_loc1+320, y_begin_loc1+70, 40, 40,color(255, 255, 0), "CAUTION", "     UnderVoltage");

  //Widget for Discharge Control.
  stroke(180);
  strokeWeight(1);
  line(x_begin_loc4, y_begin_loc4+50, x_begin_loc4 + 20, y_begin_loc4+50);
  line(x_begin_loc4 + 130, y_begin_loc4+50, x_begin_loc4 + 200, y_begin_loc4+50);
  line(x_begin_loc4, y_begin_loc4+50, x_begin_loc4, y_begin_loc4 + 735);
  line(x_begin_loc4, y_begin_loc4 + 735, x_begin_loc4 + 200, y_begin_loc4 + 735);
  line(x_begin_loc4 + 200, y_begin_loc4 + 735, x_begin_loc4 + 200, y_begin_loc4+50);

  //Widget for Status.
  stroke(180);
  strokeWeight(1);
  line(x_begin_loc2, y_begin_loc2, x_begin_loc2 + 20, y_begin_loc2);
  line(x_begin_loc2 + 125, y_begin_loc2, x_begin_loc2 + 585, y_begin_loc2);
  line(x_begin_loc2, y_begin_loc2, x_begin_loc2, y_begin_loc2 + 685);
  line(x_begin_loc2, y_begin_loc2 + 685, x_begin_loc2 + 585, y_begin_loc2 + 685);
  line(x_begin_loc2 + 585, y_begin_loc2 + 685, x_begin_loc2 + 585, y_begin_loc2);
  
  //Widgets for boxes inside Status
  BoxInsideStatus(x_begin_loc2, y_begin_loc2-10, "CHARGE");
  BoxInsideStatus(x_begin_loc2+290, y_begin_loc2-10, "CURRENT");
  BoxInsideStatus(x_begin_loc2, y_begin_loc2 + 235, "ENERGY");  
  BoxInsideStatus(x_begin_loc2 + 290, y_begin_loc2 + 235, "POWER");
  BoxInsideStatus(x_begin_loc2, y_begin_loc2 + 410, "MAX TEMP");
  BoxInsideStatus(x_begin_loc2+290, y_begin_loc2 + 410, "INDICATORS");  
  //Widget for DATA Saving.
  line(x_begin_loc3, y_begin_loc3-60, x_begin_loc3 + 20, y_begin_loc3-60);
  line(x_begin_loc3 + 60, y_begin_loc3-60, x_begin_loc3 + 400, y_begin_loc3-60);
  line(x_begin_loc3, y_begin_loc3-60, x_begin_loc3, y_begin_loc3-20);
  line(x_begin_loc3, y_begin_loc3-20, x_begin_loc3 + 400, y_begin_loc3-20);
  line(x_begin_loc3 + 400, y_begin_loc3-20, x_begin_loc3 + 400, y_begin_loc3-60);  

  //Widget for DC2350B Digonstic
  stroke(180);
  strokeWeight(1);
  line(x_begin_loc5, y_begin_loc5, x_begin_loc5 + 20, y_begin_loc5);
  line(x_begin_loc5 + 133, y_begin_loc5, x_begin_loc5 + 350, y_begin_loc5);
  line(x_begin_loc5, y_begin_loc5, x_begin_loc5, y_begin_loc5 + 160);
  line(x_begin_loc5, y_begin_loc5 + 160, x_begin_loc5 + 350, y_begin_loc5 + 160);
  line(x_begin_loc5 + 350, y_begin_loc5 + 160, x_begin_loc5 + 350, y_begin_loc5); 

  BoxInsideDC2350Diagonsis(x_begin_loc5, y_begin_loc5 - 10, "TEMP");
  BoxInsideDC2350Diagonsis(x_begin_loc5 + 150, y_begin_loc5 - 15, "REG A");  
  BoxInsideDC2350Diagonsis(x_begin_loc5 + 150, y_begin_loc5 + 30, "REG D");
  BoxInsideDC2350Diagonsis(x_begin_loc5 + 150, y_begin_loc5 + 75, "V REF");  
  
  //Widget for DC2732 Digonstic
  stroke(180);
  strokeWeight(1);
  line(x_begin_loc6, y_begin_loc6, x_begin_loc6 + 20, y_begin_loc6);
  line(x_begin_loc6 + 133, y_begin_loc6, x_begin_loc6 + 175, y_begin_loc6);
  line(x_begin_loc6, y_begin_loc6, x_begin_loc6, y_begin_loc6 + 160);
  line(x_begin_loc6, y_begin_loc6 + 160, x_begin_loc6 + 175, y_begin_loc6 + 160);
  line(x_begin_loc6 + 175, y_begin_loc6 + 160, x_begin_loc6 + 175, y_begin_loc6);
  
  BoxInsideDC2732Diagonsis(x_begin_loc6, y_begin_loc6 - 10, "BOARD TEMP");
 // BoxInsideDC2732Diagonsis(x_begin_loc6 + 175, y_begin_loc6 - 10, "INTER TEMP");

  
  //Widget for indicators.
  //stroke(180);
  //strokeWeight(1);
  //line(x_begin_loc8, y_begin_loc8+50, x_begin_loc8 + 20, y_begin_loc8+50);
  //line(x_begin_loc8 + 95, y_begin_loc8+50, x_begin_loc8 + 200, y_begin_loc8+50);
  //line(x_begin_loc8, y_begin_loc8+50, x_begin_loc8, y_begin_loc8 + 250);
  //line(x_begin_loc8, y_begin_loc8 + 250, x_begin_loc8 + 200, y_begin_loc8 + 250);
  //line(x_begin_loc8 + 200, y_begin_loc8 + 250, x_begin_loc8 + 200, y_begin_loc8+50);
  
}

void draw() {
  
  

                                  //*Intialize Connect Button Logic*\\
          //Do not try to connect unless the button is pressed, and if the button is pressed, then try\\
       //If no success in connecting, do not change any status, and if success, change button to disconnect\\
  //and assign "Connected" status to Connection Status and allow for scanning data in by making StartReading = true\\
       //if try to disconnect, immeditley disconnect and assign StartReading = false to stop reading data in\\
  if (CONNECT_Disconnect == true){

    //Only excute if button pressed once.  
    CONNECT_Disconnect = false;
    if (Try_TO_CONNECT == true){ 
        if (Serial.list().length != 0){
            serialPort.clear();
            serialPort.write("1\n");
            connect_DisConnect_Button = "Disconnect";
            CONNECTION_STATUS = "   Connected";
            Try_TO_CONNECT = false;
            StartReading = true;
            Connection_Status_Color = color(0, 255, 0);
            if (clickSoundCount1 == 0){
               ConnectSound.play();}}
    }else if (Try_TO_CONNECT == false && StartReading == true){

        ClearAllValues();
        CONNECTION_STATUS = " Not Connected";
        LoggingStatusString = "     Not Saving";
        connect_DisConnect_Button = "  Connect";
        StartReading = false;
        Try_TO_CONNECT = true;
        Connection_Status_Color = color(255,0,0);
        if (clickSoundCount == 0){
           Can_Not_Connect.play();}}
  }


                                             //Reading Data IN\\
                                //If connection success, then start scanning data in\\
  //Make sure to keep checking if there is still connection and if not change variables on screen and change startReading = false\\
                                //Split the data, parse it, and assign it to an array\\
  if (StartReading == true){

    if (Serial.list().length != 0){
        if (serialPort.available() > 0){  //Serial.list().length != 0 &&

         bufferIN = serialPort.readStringUntil('\n');
         if (bufferIN != null){
             println(bufferIN);
             SerialValues = split(bufferIN, ',');
             SerialValues = trim(SerialValues);

             //This makes sure to trim an possible empty values that may be cuase by , an extra comma 
             if (SerialValues[SerialValues.length - 1] == ""){
               SerialValues = shorten(SerialValues);
             }
      
             
             if (DC2350B == true){DC2350_Values_Reciever();}
             else if (DC2732 == true){DC2732_Values_Reciever();}
         }
        }
    }
    else{
        CONNECTION_STATUS = " Not Connected";
        connect_DisConnect_Button = "  Connect";
        StartReading = false;
        Try_TO_CONNECT = true;
        Connection_Status_Color = color(255,0,0);
        LoggingStatusString = "     Not Saving";
        Intialized = false;
        serialPort.clear();
    }
  }
  //Helps in reconnecting after dsconnecting the serial cable.
  if (Intialized == false && Try_TO_CONNECT == true){

    IntializeSerial();
  }
  
  
  




  


  //\\Updates everything in connection status widget//\\
  ConnecitonStatusUpdater();

  //\\Updates everything in DATA widget//\\
  loggingStatusUpdater();
  
  
  fill(255);
  textSize(15);
  
  //Update Colors of individual readings based on voltage values.
  voltageRangeChecker();
  
  //Contrinuosly Update the Voltage and tempreture Values 
  textFont(DigitalFont);
  updateAllBanksVoltages();
  updateAllBanksTemperatures();

  //Updates state of charge values, MAX, MIN and percentage.
  textFont(DigitalFont);
  StateOfChargeUpdater();
  StatusTEMPUpdater();
  
  //Updates DC2350B Diagonsis widget
  DC2350B_TEMP_REG_Updater(RegA, RegB, Vref);
  //Updates DC2732 Diagonsis widget
  DC2732_TEMP_REG_Updater(RegA, RegB, Vref);

  //Updates Open wire status
  checkBanksOpenWire();
 
  //Updates Discharging Status
  BatteryControlBoxAllUpdater();
  //Send to Arduino and clear all status.
  CLearingDischargeStatus();

  //Manual Toggle Dicharging buttons
 // DischargeManualToggleButton(x_begin_loc4, y_begin_loc4, true);
 
 //Updates the maximum temperature. 
 MaxTempUpdater();
 
 //Updates the indicator values
 Indicators_Updater();
 System_Status_Updater();
  //Update Consol window
  //Console_Window_Updater();
  
 //updates maximum cell voltage difference
 maximumCellVoltageDiffUpdater();
  stroke(180);strokeWeight(2);
  
  
}

void maximumCellVoltageDiffUpdater(){
  
  MaximumCellVoltage = bank_voltages[0];
  MinimumCellVotlage = bank_voltages[0];
  for (int i = 1; i < 17; i++){
    if(bank_voltages[i] > MaximumCellVoltage){
      MaximumCellVoltage = bank_voltages[i];
    }
    if(bank_voltages[i] < MinimumCellVotlage){
      MinimumCellVotlage = bank_voltages[i];
    }
  
  }
    Maximum_Cell_Difference = MaximumCellVoltage - MinimumCellVotlage;
}


void System_Status_Updater(){
  

}

void Indicators_Updater(){
  
  int i = 75;
  int j = 1;

  while(i <= 200){
    if(fault_Status[j-1] == 1){
      if (second() % 2 == 0){fill(255, 0, 0);}
      else{fill(255);}
    }
    
    else{
    fill(255);}
    
    rect(x_begin_loc8+20, y_begin_loc8+i, 30, 25);
    fill(0);
    text(j, x_begin_loc8+30, y_begin_loc8+i+15);

    fill(255);
    rect(x_begin_loc8+50, y_begin_loc8+i, 125, 25);
    
    i = i + 25;
    j = j + 1;
  }
  
  //Description 
  fill(0);textSize(12);
  text("SHORT CIRCUIT", x_begin_loc8+55, y_begin_loc8+90);
  text("OPEN CIRCUIT", x_begin_loc8+55, y_begin_loc8+115);  
//  text("SENSE O/C", x_begin_loc8+55, y_begin_loc8+140); 
  text("OVER TEMP", x_begin_loc8+55, y_begin_loc8+140); 
//  text("TEMP O/C", x_begin_loc8+55, y_begin_loc8+215); 
  text("OVER CURRENT", x_begin_loc8+55, y_begin_loc8+165);   
  text("UNDER VOLT", x_begin_loc8+55, y_begin_loc8+190); 
  text("OVER VOLT", x_begin_loc8+55, y_begin_loc8+215); 
  //text("DC2350B FAULT", x_begin_loc8+55, y_begin_loc8+315); 
  //text("DC2732 FAULT", x_begin_loc8+55, y_begin_loc8+340); 

}

void MaxTempUpdater(){

  MAXIMUM_TEMP = bank_temperature_array[0];
  for (int i = 1; i <= 17; i++){
    if (bank_temperature_array[i] > MAXIMUM_TEMP){
       MAXIMUM_TEMP = bank_temperature_array[i];
   }
  }
}

void IntializeSerial(){
  
  //This will only may cause error if board is not connected before running the application.  
  try{
     serialPort = new Serial(this, Serial.list()[0], BAUD_RATE);
     serialPort.bufferUntil('\n');
     Intialized = true;
  } catch (RuntimeException e){
    
    if (e.getMessage().contains("<init>")){
        Intialized = false; 
    }
  }
}

void checkBanksOpenWire(){

  if (OPEN_WIRE_ARRAY[0] == true){image(OpenWireImage, x_begin_loc + 40, y_begin_loc + 60, 35, 40);}
  if (OPEN_WIRE_ARRAY[1] == true){image(OpenWireImage, x_begin_loc + 40, y_begin_loc + 170, 35, 40);}
  if (OPEN_WIRE_ARRAY[2] == true){image(OpenWireImage, x_begin_loc + 40, y_begin_loc + 280, 35, 40);}
  if (OPEN_WIRE_ARRAY[3] == true){image(OpenWireImage, x_begin_loc + 40, y_begin_loc + 390, 35, 40);}
  if (OPEN_WIRE_ARRAY[4] == true){image(OpenWireImage, x_begin_loc + 40, y_begin_loc + 500, 35, 40);}
  if (OPEN_WIRE_ARRAY[5] == true){image(OpenWireImage, x_begin_loc + 40, y_begin_loc + 610, 35, 40);}
  
  if (OPEN_WIRE_ARRAY[6] == true){image(OpenWireImage, x_begin_loc + 230, y_begin_loc + 60, 35, 40);}
  if (OPEN_WIRE_ARRAY[7] == true){image(OpenWireImage, x_begin_loc + 230, y_begin_loc + 170, 35, 40);}
  if (OPEN_WIRE_ARRAY[8] == true){image(OpenWireImage, x_begin_loc + 230, y_begin_loc + 280, 35, 40);}
  if (OPEN_WIRE_ARRAY[9] == true){image(OpenWireImage, x_begin_loc + 230, y_begin_loc + 390, 35, 40);}
  if (OPEN_WIRE_ARRAY[10] == true){image(OpenWireImage, x_begin_loc + 230, y_begin_loc + 500, 35, 40);}
  if (OPEN_WIRE_ARRAY[11] == true){image(OpenWireImage, x_begin_loc + 230, y_begin_loc + 610, 35, 40);}
  
  if (OPEN_WIRE_ARRAY[12] == true){image(OpenWireImage, x_begin_loc + 430, y_begin_loc + 60, 35, 40);}
  if (OPEN_WIRE_ARRAY[13] == true){image(OpenWireImage, x_begin_loc + 430, y_begin_loc + 170, 35, 40);}
  if (OPEN_WIRE_ARRAY[14] == true){image(OpenWireImage, x_begin_loc + 430, y_begin_loc + 280, 35, 40);}
  if (OPEN_WIRE_ARRAY[15] == true){image(OpenWireImage, x_begin_loc + 430, y_begin_loc + 390, 35, 40);}
  if (OPEN_WIRE_ARRAY[16] == true){image(OpenWireImage, x_begin_loc + 430, y_begin_loc + 500, 35, 40);}
  if (OPEN_WIRE_ARRAY[17] == true){image(OpenWireImage, x_begin_loc + 430, y_begin_loc + 610, 35, 40);}

}
void BatteryControlBoxAllUpdater(){
  
  fill(0);
  rect(x_begin_loc4 + 35, y_begin_loc4 + 695, 140, 20);
  fill(255);
  stroke(180);
  strokeWeight(1);
  rect(x_begin_loc4 + 48, y_begin_loc4 + 79, 114, 24, 4); 
  ClearDischargeButton();  
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 140, Dicharge_Status[0], OverDischargeButtonsArray[0]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 170, Dicharge_Status[1], OverDischargeButtonsArray[1]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 200, Dicharge_Status[2], OverDischargeButtonsArray[2]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 230, Dicharge_Status[3], OverDischargeButtonsArray[3]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 260, Dicharge_Status[4], OverDischargeButtonsArray[4]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 290, Dicharge_Status[5], OverDischargeButtonsArray[5]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 320, Dicharge_Status[6], OverDischargeButtonsArray[6]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 350, Dicharge_Status[7], OverDischargeButtonsArray[7]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 380, Dicharge_Status[8], OverDischargeButtonsArray[8]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 410, Dicharge_Status[9], OverDischargeButtonsArray[9]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 440, Dicharge_Status[10], OverDischargeButtonsArray[10]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 470, Dicharge_Status[11], OverDischargeButtonsArray[11]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 500, Dicharge_Status[12], OverDischargeButtonsArray[12]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 530, Dicharge_Status[13], OverDischargeButtonsArray[13]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 560, Dicharge_Status[14], OverDischargeButtonsArray[14]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 590, Dicharge_Status[15], OverDischargeButtonsArray[15]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 620, Dicharge_Status[16], OverDischargeButtonsArray[16]);
  BatteryControlBoxUpdater(x_begin_loc4 + 15, y_begin_loc4 + 650, Dicharge_Status[17], OverDischargeButtonsArray[17]);
  
  fill(255);
  text(""+(Maximum_Cell_Difference != 0 ? nf(Maximum_Cell_Difference, 1, 2) : "--") + " V", x_begin_loc4 + 90, y_begin_loc4 + 710);

}


void BatteryControlBoxUpdater(int x, int y, boolean STATUS, boolean overtheButton){
  
  if (STATUS == false){
    stroke(255);
    strokeWeight(1);
    fill(255, 128, 128);
    circle(x+85, y+10, 14);
    noStroke();
    fill(255, 0, 0);
    circle(x+85.5, y+10.5, 8);  
    
    stroke(223);
    strokeWeight(0.5);
    fill(255);
    rect(x+120, y+4, 50, 12, 100, 100, 100, 100);
    stroke(210);
    strokeWeight(1.1);
    rect(x+120, y+5, 25, 10, 100, 100, 100, 100);
    
    if (overtheButton == true){
        fill(36, 160, 237);
        rect(x+122.5, y+6, 20, 8, 100, 100, 100, 100);
    }
    
  }else{
    stroke(255);
    strokeWeight(1);
    fill(128, 230, 128);
    circle(x+85, y+10, 14);
    noStroke();
    fill(78, 209, 100);
    circle(x+85.5, y+10.5, 8);
    
    stroke(223);
    strokeWeight(1);
    fill(78, 209, 100);
    rect(x+120, y+4, 50, 12, 100, 100, 100, 100);
    stroke(78, 209, 100);
    strokeWeight(1);
    fill(255);
    rect(x+144, y+5, 25, 10, 100, 100, 100, 100);
  }
  
}


void loggingStatusUpdater(){
  fill(255);
  rect(x_begin_loc3 + 65, y_begin_loc3 - 55, 125, 30, 4);
  rect(x_begin_loc3 + 240, y_begin_loc3 - 55, 125, 30, 4);

  fill(0);
  stroke(180);strokeWeight(2);
  rect(x_begin_loc3 + 70, y_begin_loc3 - 49, 115, 20, 4);
  rect(x_begin_loc3 + 245, y_begin_loc3 - 49, 115, 20, 4);

  
  if (ProgressIndicator >= 2*PI){
    ProgressIndicator = 0;
  }
  else{
    ProgressIndicator = ProgressIndicator + PI/16;
  }
  if (LoggingStatusString.equals("     Not Saving")){
    fill(255, 0, 0);}
  else{
    noStroke();
    fill(0, 255, 0);
    arc(x_begin_loc3+175, y_begin_loc3 - 40, 10, 10, ProgressIndicator, PI+ProgressIndicator, PIE);
    fill(0);
    circle(x_begin_loc3+175,  y_begin_loc3 - 40, 5);
    fill(0, 255, 0);
    
  }
  textSize(12);
  text(nf(hour(), 2, 0)+ ":" + nf(minute(), 2, 0) + ":" + nf(second(), 2, 0), x_begin_loc3 + 280, y_begin_loc3-35);
  text(LoggingStatusString, x_begin_loc3 + 75, y_begin_loc3-35);
}


void ConnecitonStatusUpdater(){
  
  fill(0);
  rect(x_begin_loc + 128, y_begin_loc - 45, 92, 20, 4);
  rect(x_begin_loc + 269, y_begin_loc - 45, 40, 20, 4);
  rect(x_begin_loc + 380, y_begin_loc - 45, 45, 20, 4);
  textFont(Default);
  textSize(12);
  fill(Connection_Status_Color);
  text(CONNECTION_STATUS, x_begin_loc + 130, y_begin_loc - 30);
  fill(255);
  text(BAUD_RATE, x_begin_loc + 382, y_begin_loc - 30);
  text(PORT, x_begin_loc + 270, y_begin_loc - 30);
  fill(255);
  stroke(180);
  strokeWeight(1);
  rect(x_begin_loc + 508, y_begin_loc -48 , 86, 24, 4);  
  connectButton();
  checkBox();
  if (DC2350B){   //Updates Baud Rate based on board selection
    fill(0);
    BAUD_RATE = 115200;
    circle(x_begin_loc + 450, y_begin_loc - 30, 7);
  }else if(DC2732){
    fill(0);
    BAUD_RATE = 250000;
    circle(x_begin_loc + 450, y_begin_loc - 45, 7);
  }
}



void ClearAllValues(){

  //Will do these in for loop
  bank_voltages[0] = 0;
  bank_voltages[1] = 0;
  bank_voltages[2] = 0;
  bank_voltages[3] = 0;
  bank_voltages[4] = 0;
  bank_voltages[5] = 0;
  bank_voltages[6] = 0;
  bank_voltages[7] = 0;
  bank_voltages[8] = 0;
  bank_voltages[9] = 0;
  bank_voltages[10] = 0;
  bank_voltages[11] = 0;
  bank_voltages[12] = 0;
  bank_voltages[13] = 0;
  bank_voltages[14] = 0;
  bank_voltages[15] = 0;
  bank_voltages[16] = 0;
  bank_voltages[17] = 0;
  
  SOC = 0;
  CHIP_TEMP = 0;
  CHIP_TEMP1 = 0;
  RegA = 0;
  RegB = 0;
  ENERGY = 0;
  POWER = 0;
  Vref = 0;
  CURRENT = 0;
  MAXIMUM_TEMP = 0;
  
  for (int l = 0; l < fault_Status.length; l++){
     fault_Status[l] = 0;
  }
  //reset temp values
  for (int j = 0; j < bank_temperature_array.length; j++){
     bank_temperature_array[j] = 0;
  }
  //reset battery banks color.
  for (int k = 0; k < bank_color_array.length; k++){
     bank_color_array[k] = color(255);
     }

  //reset Open wire.
  for (int i = 0; i < OPEN_WIRE_ARRAY.length; i++){
     OPEN_WIRE_ARRAY[i] = false;}
     
}


void SetDischargeStatus(){
    for (int i = 0; i<=17; i++){
      if (SET_DISCHARGE[i] == true && StartReading == true){
        println(SET_DISCHARGE[i]);
        
        serialPort.write("m\n");
        serialPort.write("2\n");
        serialPort.write((i+1) + "\n");  
  
        SET_DISCHARGE[i] = false;
        Dicharge_Status[i]  = true;     

        serialPort.write("m\n");
        serialPort.write(1 + "\n"); // back to loop

   } //set dischar true
  } //for 
}

void CLearingDischargeStatus(){
  
  if(CLEAR_DISCHARGE == true && StartReading == true){ //
   
    if (serialPort.available() > 0){

        serialPort.write("m\n");
        serialPort.write("3\n");
        myString = serialPort.readStringUntil('\n');
        
        if (myString != null){
         
           int myInt = int(myString.trim());
            if ( myInt == 9){
              //clear all status.
              CLEAR_DISCHARGE = false;      //Make sure to stop clearing untill the button is pressed again. 
              for(int i = 0; i <= Dicharge_Status.length - 1; i++){    
                 Dicharge_Status[i] = false;}
             
             serialPort.write("1\n");
            }
        }   
    }
  }
}


//To Exit using Escape. 
void keyPressed() {
  
  if (key == ESC){
     exit();
  }
}

void mousePressed(){
  
  if (overConnectButton) {
    CONNECT_Disconnect = true; 
  }else {
    CONNECT_Disconnect = false;
  } 

 if (overClearDischargeButton) {
    CLEAR_DISCHARGE = true; 
  }else {
    CLEAR_DISCHARGE = false;
  } 
  
  if (overDC2350B_Option){
    DC2350B = true;
    DC2732 = false;
  }
  if (overDC2732_Option){
    DC2732 = true;
    DC2350B = false;
  }
  
  for (int i = 0; i<=17; i++){
    if (OverDischargeButtonsArray[i] == true){
       SET_DISCHARGE[i] = true;
    }
    else{
       SET_DISCHARGE[i] = false; 
    }

  }
  
  SetDischargeStatus();

 
}


void DC2350_Values_Reciever(){
  
    //This if there are no open wires  // && SerialValues.length <= 52){ 

    if (SerialValues.length == 55){
       //SAVE DATA FIRST
       SAVE_DC2350_Values();

       //Cell Voltages
       bank_voltages[0] = float(SerialValues[0]);
       bank_voltages[1] = float(SerialValues[1]);
       bank_voltages[2] = float(SerialValues[2]);
       bank_voltages[3] = float(SerialValues[3]);
       bank_voltages[4] = float(SerialValues[4]);
       bank_voltages[5] = float(SerialValues[5]);
       bank_voltages[6] = float(SerialValues[6]);
       bank_voltages[7] = float(SerialValues[7]);
       bank_voltages[8] = float(SerialValues[8]);
       bank_voltages[9] = float(SerialValues[9]);
       bank_voltages[10] = float(SerialValues[10]);// - 0.3;  //FIX VOLTAGE READINGS
       bank_voltages[11] = float(SerialValues[11]);// + 0.6;
       bank_voltages[12] = float(SerialValues[12]);// - 0.3;
       bank_voltages[13] = float(SerialValues[13]);
       bank_voltages[14] = float(SerialValues[14]);
       bank_voltages[15] = float(SerialValues[15]);
       bank_voltages[16] = float(SerialValues[16]);
       bank_voltages[17] = float(SerialValues[17]);
       
       //DC2350B specific Values. 
       SOC = float(SerialValues[18]);
       CHIP_TEMP = float(SerialValues[19]);
       RegA = float(SerialValues[20]);
       RegB = float(SerialValues[21]);
       Vref = float(SerialValues[22]);
       
       //Temp Values
       bank_temperature_array[16] = float(SerialValues[23]);
       bank_temperature_array[17] = float(SerialValues[24]);
       for (int j = 25; j < 41; j++){
          bank_temperature_array[j - 25] = float(SerialValues[j]);
       }
       
       CURRENT = float(SerialValues[41]);
       POWER = float(SerialValues[42]);
       VBAT = float(SerialValues[43]);
       
       CHIP_TEMP1 = float(SerialValues[44]);
       Internal_TEMP =  float(SerialValues[45]); 
       
       FAST_CURRENT = float(SerialValues[46]);       
       FAST_BAT = float(SerialValues[47]);

      //Fault codes
      for (int l = 0; l < fault_Status.length; l++){
        fault_Status[l] = int(SerialValues[48 + l]);
      }
    


      }
      
     // println(SerialValues.length);
    //  if (SerialValues.length > 55 && SerialValues.length < ){
      //Open Wire possible values.
      // if (SerialValues.length > 32){
      // for (int i = SerialValues.length - 1;i > 31 ;i--){       
      //     OPEN_WIRE_ARRAY[int(SerialValues[i]) - 1] = true; 
      // }
      //}      
    //  }
}

void DC2732_Values_Reciever(){
  
  

}

void ClearDischargeButton(){
      updateMouse();
      if (overClearDischargeButton) {
         fill(36, 160, 237);
         
         rect(x_begin_loc4 + 50, y_begin_loc4 + 82 , 110, 18, 4);
         textSize(14.4);
      } else {
        textSize(14);
      }
      fill(0);
      text("Clear All", x_begin_loc4 + 65, y_begin_loc4 + 95);
      noStroke();
}

void connectButton(){
      updateMouse();
      if (overConnectButton) {
         fill(36, 160, 237);
         
         rect(x_begin_loc + 511, y_begin_loc -45 , 80, 18, 4);
         textSize(14.4);
      } else {
        textSize(14);
      }
      fill(0);
      text(connect_DisConnect_Button, x_begin_loc + 515, y_begin_loc -30);
      noStroke();
}


void SAVE_DC2350_Values(){
 

  LoggingStatusString = "    Saving";
//  if ((second() % 4) == 0){
 // if (bufferIN){
  SerialOutput.print(str(hour())+ ":" + str(minute())+ ":" + str(second()) + ", ");
  SerialOutput.println(bufferIN);
      //for (int i = 0; i <= SerialValues.length - 1; ++i){
      //  if (i != SerialValues.length-1){
      //   SerialOutput.print(SerialValues[i] + ", ");}
        
      // else{
      //   SerialOutput.print(SerialValues[i]);
      // }
      //}
  // }
// }
}

void checkBox(){
      updateMouse();
      fill(255);
      stroke(180);
      strokeWeight(1);
      circle(x_begin_loc + 450, y_begin_loc - 30, 10);
      circle(x_begin_loc + 450, y_begin_loc - 45, 10);
}

boolean overConnectButton(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    
    return true;
  } else {
    return false;
  }
}

boolean overClearDischargeButton(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    
    return true;
  } else {
    return false;
  }
}


boolean OverDischargeButtons(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    
    return true;
  } else {
    return false;
  }
}

boolean overCheckBox(int x, int y, int diameter) {
  float disX = x - mouseX;
  float disY = y - mouseY;
  if(sqrt(sq(disX) + sq(disY)) < diameter/2 ) {
    return true;
  } else {
    return false;
  }
}


void updateMouse() {
  
  if ( overConnectButton(x_begin_loc + 510, y_begin_loc -46 , 72, 20) ) {
    overConnectButton = true;
   
  }else if (overCheckBox(x_begin_loc + 450, y_begin_loc - 30, 12)){
    overDC2350B_Option = true;
    overDC2732_Option = false; 
  }else if (overCheckBox(x_begin_loc + 450, y_begin_loc - 45, 12)){
    overDC2350B_Option = false;
    overDC2732_Option = true;
  }
  //Discharge and clear discharge Buttons 
  else if (overClearDischargeButton(x_begin_loc4 + 50, y_begin_loc4 + 82 , 110, 18)){overClearDischargeButton = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+144, 50, 12)){OverDischargeButtonsArray[0] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+174, 50, 12)){OverDischargeButtonsArray[1] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+204, 50, 12)){OverDischargeButtonsArray[2] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+234, 50, 12)){OverDischargeButtonsArray[3] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+264, 50, 12)){OverDischargeButtonsArray[4] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+294, 50, 12)){OverDischargeButtonsArray[5] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+324, 50, 12)){OverDischargeButtonsArray[6] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+354, 50, 12)){OverDischargeButtonsArray[7] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+384, 50, 12)){OverDischargeButtonsArray[8] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+414, 50, 12)){OverDischargeButtonsArray[9] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+444, 50, 12)){OverDischargeButtonsArray[10] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+474, 50, 12)){OverDischargeButtonsArray[11] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+504, 50, 12)){OverDischargeButtonsArray[12] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+534, 50, 12)){OverDischargeButtonsArray[13] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+564, 50, 12)){OverDischargeButtonsArray[14] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+594, 50, 12)){OverDischargeButtonsArray[15] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+624, 50, 12)){OverDischargeButtonsArray[16] = true;}
  else if (OverDischargeButtons(x_begin_loc4+135, y_begin_loc4+654, 50, 12)){OverDischargeButtonsArray[17] = true;}


  else {
    overConnectButton = false;
    overDC2350B_Option = false;
    overDC2732_Option = false;
    overClearDischargeButton = false;
    OverDischargeButtonsArray[0] = false;
    OverDischargeButtonsArray[1] = false;
    OverDischargeButtonsArray[2] = false;
    OverDischargeButtonsArray[3] = false;
    OverDischargeButtonsArray[4] = false;
    OverDischargeButtonsArray[5] = false;
    OverDischargeButtonsArray[6] = false;
    OverDischargeButtonsArray[7] = false;
    OverDischargeButtonsArray[8] = false;
    OverDischargeButtonsArray[9] = false;
    OverDischargeButtonsArray[10] = false;
    OverDischargeButtonsArray[11] = false;
    OverDischargeButtonsArray[12] = false;
    OverDischargeButtonsArray[13] = false;
    OverDischargeButtonsArray[14] = false;
    OverDischargeButtonsArray[15] = false;
    OverDischargeButtonsArray[16] = false;
    OverDischargeButtonsArray[17] = false;
  }
}

//Updates Battery Bank Figures
void BatteryBankVoltageUpdater(int x, int y, float voltageValue, color BatteryColor){

  stroke(180);strokeWeight(2);
  fill(0);
  textSize(18);
  rect(x+89, y+59, 85-8, 40-8);
  noStroke();
  fill(255);
  //text(nf(voltageValue, 1, 2) + " V", x + 95, y + 80);
  text(""+(voltageValue != 0 ? nf(voltageValue, 1, 2) : "--") + " V", x + 95, y + 80);
  

  

  //Update the Battery Level.
  if (voltageValue >= 0 && (56.0 - 56.0*voltageValue/MAX_VOLTAGE) >= -0.1){
     if (voltageValue == 0){
       fill(BatteryColor);
       rect(x + 12, (float)(y + 35.0) + (56.0-56.0*MAX_VOLTAGE/MAX_VOLTAGE), 56.0, 56.0*(MAX_VOLTAGE/MAX_VOLTAGE));
     }else{    
       fill(255);
       rect(x + 12, (float)(y + 35.0) + (56.0-56.0*MAX_VOLTAGE/MAX_VOLTAGE), 56.0, 56.0*(MAX_VOLTAGE/MAX_VOLTAGE));
       fill(BatteryColor);
       rect(x + 12, (float)(y + 35.0) + (56.0-56.0*voltageValue/MAX_VOLTAGE), 56.0, 56.0*(voltageValue/MAX_VOLTAGE));
     }
  }
}


void DC2350B_TEMP_REG_Updater(float value1, float value2, float value3){
  
    strokeWeight(2);
    stroke(180);
    
    if (CHIP_TEMP >= MAX_CHIP_TEMP){
      fill(255, 50, 0);}
    else if (CHIP_TEMP < MAX_CHIP_TEMP && CHIP_TEMP > MIN_CHIP_TEMP){
      fill(56, 118, 29);
    }
    else if (CHIP_TEMP < MIN_CHIP_TEMP && CHIP_TEMP > 0){
      fill(210, 232, 242);
    }
    else{
      fill(255);
    }
    rect(x_begin_loc5 + 77.5, y_begin_loc5+35, 15, 50,  40, 40, 40, 40);
    circle(x_begin_loc5+85, y_begin_loc5 +105, 50);
    
    textSize(12);
    fill(0);
    text(""+ (CHIP_TEMP > 0 ? (int)CHIP_TEMP : "--") + "\u00B0C", x_begin_loc5+70, y_begin_loc5 +110);

    //RegA
    fill(0);
    rect(x_begin_loc5+175, y_begin_loc5+25, 150, 20);
    fill(255);
    text(""+(value1 != 0 ? nf(value1, 1, 2) : "--") + " V", x_begin_loc5 + 235, y_begin_loc5 + 38);
    //RegD
    fill(0);
    rect(x_begin_loc5+175, y_begin_loc5+70, 150, 20);
    fill(255);
    text(""+(value2 != 0 ? nf(value2, 1, 2) : "--") + " V", x_begin_loc5 + 235, y_begin_loc5 + 83);
    //Vref
    fill(0);
    rect(x_begin_loc5+175, y_begin_loc5+115, 150, 20);
    fill(255);
    text(""+(value3 != 0 ? nf(value3, 1, 2) : "--") + " V", x_begin_loc5 + 235, y_begin_loc5 + 128);
}

void DC2732_TEMP_REG_Updater(float value1, float value2, float value3){
  
    //Board Temp
    strokeWeight(2);
    stroke(180);
    
    if (Internal_TEMP >= MAX_CHIP_TEMP){
      fill(255, 50, 0);}
    else if (Internal_TEMP < MAX_CHIP_TEMP && Internal_TEMP > MIN_CHIP_TEMP){
      fill(56, 118, 29);
    }
    else if (Internal_TEMP < MIN_CHIP_TEMP && Internal_TEMP > 0){
      fill(210, 232, 242);
    }
    else{
      fill(255);
    }
    rect(x_begin_loc6 + 77.5, y_begin_loc6+35, 15, 50,  40, 40, 40, 40);
    circle(x_begin_loc6+85, y_begin_loc6 +105, 50);
    
    textSize(12);
    fill(0);
    text(""+ (Internal_TEMP > 0 ? (int)Internal_TEMP : "--") + "\u00B0C", x_begin_loc6+70, y_begin_loc6 +110);

    //Internal Temp
    //strokeWeight(2);
    //stroke(180);
    
    //if (Internal_TEMP >= MAX_CHIP_TEMP){
    //  fill(255, 50, 0);}
    //else if (Internal_TEMP < MAX_CHIP_TEMP && Internal_TEMP > MIN_CHIP_TEMP){
    //  fill(56, 118, 29);
    //}
    //else if (Internal_TEMP < MIN_CHIP_TEMP && Internal_TEMP > 0){
    //  fill(210, 232, 242);
    //}
    //else{
    //  fill(255);
    //}
    //rect(x_begin_loc6 + 77.5+175, y_begin_loc6+35, 15, 50,  40, 40, 40, 40);
    //circle(x_begin_loc6+85+175, y_begin_loc6 +105, 50);
    
    //textSize(12);
    //fill(0);
    //text(""+ (Internal_TEMP > 0 ? (int)Internal_TEMP : "--") + "\u00B0C", x_begin_loc6+70+175, y_begin_loc6 +110);
}


void StatusTEMPUpdater(){
    fill(0);
    


}


void StateOfChargeUpdater(){
  
  
  //Calculte ENERGY 
  ENERGY = 2.5*SOC; 
  
  //Charge
  strokeWeight(2);
  stroke(180);
  fill(0);
  textSize(18);
  rect(x_begin_loc2+ 37.5, y_begin_loc2 +205, 100, 20);
  rect(x_begin_loc2+ 157.5, y_begin_loc2 +205, 100, 20); 

  if (SOC > 0){
    PercentageOfSOC = (SOC-MIN_SOC_VOLTAGE)/(MAX_SOC_VOLTAGE-MIN_SOC_VOLTAGE)*100;
  }
  else{
    PercentageOfSOC = 0;
  }

  stroke(240);
  strokeWeight(1);
  fill(240);
  arc(x_begin_loc2+155, y_begin_loc2 +115, 150, 150, -PI/2, -PI/2 + 2*PI, PIE); 
  
  if (SOC >= MIN_SOC_VOLTAGE && SOC <= MAX_SOC_VOLTAGE){
     fill(color(56, 118, 29));
  }else if (SOC == 0){
     fill(255);
  }
  else if(SOC < MIN_SOC_VOLTAGE){
     fill(color(255, 255, 0));
  }
  else if (SOC > MAX_SOC_VOLTAGE){
     fill(color(255, 192, 192));
  }

  
  stroke(180);
  strokeWeight(1);
  arc(x_begin_loc2+155, y_begin_loc2 +115, 150, 150, -PI/2, -PI/2 + 2*PI*PercentageOfSOC/100.0, PIE);

  if (SOC == 0){
     noStroke();
     fill(240,240,240);
     arc(x_begin_loc2+155, y_begin_loc2 +115, 150, 150, -PI/2, -PI/2 + 2*PI, PIE);  
  }

  stroke(180);
  strokeWeight(1);
  fill(255);
  circle(x_begin_loc2+155, y_begin_loc2 +115, 110);
  textSize(14);
  fill(255);
  text("MAX:" + nf(MAX_SOC_VOLTAGE, 2, 1)+ " V", x_begin_loc2 + 167.5, y_begin_loc2 + 220);
  text("MIN:" + nf(MIN_SOC_VOLTAGE, 2, 1)+ " V", x_begin_loc2 + 47.5, y_begin_loc2 + 220);
  fill(0);
  textSize(25);  
  if (PercentageOfSOC >= 0){
    text(""+(PercentageOfSOC != 0 ? nf(PercentageOfSOC, 2, 1) + "%" : "  --"), x_begin_loc2+110, y_begin_loc2 +120);
    textSize(12);
    text(""+(SOC != 0 ? nf(SOC, 2, 1) + " V" : ""), x_begin_loc2+135, y_begin_loc2 +140);
    textSize(25);  
}
  
  
  
  
  //Current
  fill(255);stroke(180);strokeWeight(1);
  rect(x_begin_loc2 + 340, y_begin_loc2 + 150-60, 195, 70);

  fill(0);stroke(180);strokeWeight(2);
  rect(x_begin_loc2 + 350, y_begin_loc2 + 300-200, 175, 50);
  fill(255);
  textSize(20);
  if (CURRENT >= 0){
    text(""+(CURRENT != 0 ? nf(CURRENT, 2, 1) + " A" : "  --"), x_begin_loc2+400, y_begin_loc2 +130);
  }    
  //stroke(180);
  //fill(240);
  //arc(x_begin_loc2+440, y_begin_loc2 +165, 175, 150, -PI, 0, OPEN);
    
  //pushMatrix(); //saving current transformation matrix onto the matrix stack
  //imageMode(CORNER); //draw image using corner mode
  //translate((x_begin_loc2)+440, (y_begin_loc2)+160); //translating "(width/2)+2, (height/2)+15" to "0,0"
  //rotate(((HALF_PI)-19.69)+CURRENT); //rotation needle image to position to zero in meter image 
  //image(needle, 0, 0, 60, 50); //drawing needle image
  //popMatrix();//removing the current transformation matrix off the matrix stack
  
  //fill(240);noStroke();
  //rect(x_begin_loc2 + 325, y_begin_loc2+160, 200, 70);
  
  

    
   
  
  //Power
  fill(255);strokeWeight(1);
  rect(x_begin_loc2 + 340, y_begin_loc2 + 290, 195, 70);
  rect(x_begin_loc2+ 322.5, y_begin_loc2 +370, 110, 30);
  rect(x_begin_loc2+ 442.5, y_begin_loc2 +370, 110, 30);
  fill(0);stroke(180);
  strokeWeight(1);
  rect(x_begin_loc2+ 327.5, y_begin_loc2 +375, 100, 20);
  rect(x_begin_loc2+ 447.5, y_begin_loc2 +375, 100, 20); 
  stroke(180);strokeWeight(3);
  rect(x_begin_loc2 + 350, y_begin_loc2 + 300, 175, 50);
  fill(255);
  textSize(12);
  text("MAX:" + nf(MAX_POWER, 2, 1)+ " W", x_begin_loc2 + 451, y_begin_loc2 + 390);
  text("MIN:" + nf(MIN_POWER, 2, 1)+ " W", x_begin_loc2 + 341, y_begin_loc2 + 390);
  textSize(20);
  if (POWER >= 0){
    text(""+(POWER != 0 ? nf(POWER, 2, 1) + " W" : "  --"), x_begin_loc2+390, y_begin_loc2 +330);
  }  
  
  //Energy
  strokeWeight(1);
  fill(255);
  rect(x_begin_loc2 + 50, y_begin_loc2 + 290, 195, 70);
  rect(x_begin_loc2+ 32.5, y_begin_loc2 +370, 110, 30);
  rect(x_begin_loc2+ 152.5, y_begin_loc2 +370, 110, 30); 
  fill(0);stroke(180);strokeWeight(2);
  rect(x_begin_loc2+ 37.5, y_begin_loc2 +375, 100, 20);
  rect(x_begin_loc2+ 157.5, y_begin_loc2 +375, 100, 20); 
  textSize(12);
  fill(255);
  text("MAX:" + nf(MAX_ENERGY, 2, 1)+ " Wh", x_begin_loc2 + 167.5, y_begin_loc2 + 390);
  text("MIN:" + nf(MIN_ENERGY, 2, 1)+ " Wh", x_begin_loc2 + 47.5, y_begin_loc2 + 390);
  
  textSize(20);   
  fill(0);
  stroke(180);strokeWeight(3);
  rect(x_begin_loc2 + 60, y_begin_loc2 + 300, 175, 50);
  fill(255);
  if (ENERGY >= 0){
    text(""+(ENERGY != 0 ? nf(ENERGY, 2, 1) + " Wh" : "--"), x_begin_loc2+110, y_begin_loc2 +330);
  }
  
  
  //Maximum Temperature
  fill(255);strokeWeight(1);
  rect(x_begin_loc2+ 32.5, y_begin_loc2 +620, 110, 30);
  rect(x_begin_loc2+ 152.5, y_begin_loc2 +620, 110, 30); 
  fill(0);stroke(180);strokeWeight(2);
  rect(x_begin_loc2+ 37.5, y_begin_loc2 +625, 100, 20);
  rect(x_begin_loc2+ 157.5, y_begin_loc2 +625, 100, 20); 
  
  if (MAXIMUM_TEMP >= MAX_CHIP_TEMP){
    fill(255, 50, 0);}
  else if (MAXIMUM_TEMP < MAX_CHIP_TEMP && MAXIMUM_TEMP > MIN_CHIP_TEMP){
    fill(56, 118, 29);
  }
  else if (MAXIMUM_TEMP < MIN_CHIP_TEMP && MAXIMUM_TEMP > 0){
    fill(210, 232, 242);
  }
  else{
    fill(255);
  }
  rect(x_begin_loc2 + 141.5, y_begin_loc2+ 460, 20, 75,  40, 40, 40, 40);
  circle(x_begin_loc2+150, y_begin_loc2 +565, 75);

 
  fill(255);
  textSize(12);
  text("MAX:" + nf(MAX_CHIP_TEMP, 2, 0)+ " \u00B0C", x_begin_loc2 + 170, y_begin_loc2 + 640);
  text("MIN:" + nf(MIN_CHIP_TEMP, 2, 0)+ " \u00B0C", x_begin_loc2 + 50, y_begin_loc2 + 640);

  textSize(15);
  fill(0);

  text(""+ (MAXIMUM_TEMP > 0 ? (int)MAXIMUM_TEMP : "--") + "\u00B0C", x_begin_loc2+130, y_begin_loc2 +570);
  
}

//Updates Bank temperatures
void BatteryBankTemperatureUpdater(int x, int y, float temperatureValue){
  
  fill(0);
  textSize(18);
  stroke(180);strokeWeight(2);
  rect(x+89, y+19, 85-8, 35-8);
  noStroke();
  fill(255);
  //text(nf(temperatureValue, 2, 1) + " C", x + 95, y + 40);
  text(""+((int) temperatureValue != 0 ? nf((int) temperatureValue, 2, 0) : "--") + " \u00B0C", x + 95, y + 37.5);
  
}

void updateAllBanksVoltages (){

  BatteryBankVoltageUpdater(x_begin_loc + 20, y_begin_loc + 20, bank_voltages[0], bank_color_array[0]);
  BatteryBankVoltageUpdater(x_begin_loc + 20, y_begin_loc + 130, bank_voltages[1], bank_color_array[1]);
  BatteryBankVoltageUpdater(x_begin_loc + 20, y_begin_loc + 240, bank_voltages[2], bank_color_array[2]);
  BatteryBankVoltageUpdater(x_begin_loc + 20, y_begin_loc + 350, bank_voltages[3], bank_color_array[3]);
  BatteryBankVoltageUpdater(x_begin_loc + 20, y_begin_loc + 460, bank_voltages[4], bank_color_array[4]);
  BatteryBankVoltageUpdater(x_begin_loc + 20, y_begin_loc + 570, bank_voltages[5], bank_color_array[5]);

  BatteryBankVoltageUpdater(x_begin_loc + 210, y_begin_loc + 20, bank_voltages[6], bank_color_array[6]);
  BatteryBankVoltageUpdater(x_begin_loc + 210, y_begin_loc + 130, bank_voltages[7], bank_color_array[7]);
  BatteryBankVoltageUpdater(x_begin_loc + 210, y_begin_loc + 240, bank_voltages[8], bank_color_array[8]);
  BatteryBankVoltageUpdater(x_begin_loc + 210, y_begin_loc + 350, bank_voltages[9], bank_color_array[9]);
  BatteryBankVoltageUpdater(x_begin_loc + 210, y_begin_loc + 460, bank_voltages[10], bank_color_array[10]);
  BatteryBankVoltageUpdater(x_begin_loc + 210, y_begin_loc + 570, bank_voltages[11], bank_color_array[11]);
  
  BatteryBankVoltageUpdater(x_begin_loc + 410, y_begin_loc + 20, bank_voltages[12], bank_color_array[12]);    
  BatteryBankVoltageUpdater(x_begin_loc + 410, y_begin_loc + 130, bank_voltages[13], bank_color_array[13]);    
  BatteryBankVoltageUpdater(x_begin_loc + 410, y_begin_loc + 240, bank_voltages[14], bank_color_array[14]);   
  BatteryBankVoltageUpdater(x_begin_loc + 410, y_begin_loc + 350, bank_voltages[15], bank_color_array[15]);   
  BatteryBankVoltageUpdater(x_begin_loc + 410, y_begin_loc + 460, bank_voltages[16], bank_color_array[16]);   
  BatteryBankVoltageUpdater(x_begin_loc + 410, y_begin_loc + 570, bank_voltages[17], bank_color_array[17]);
  
  //Update Batteries Level and Color.

}


void updateAllBanksTemperatures (){

  BatteryBankTemperatureUpdater(x_begin_loc + 20, y_begin_loc + 20, bank_temperature_array[0]);
  BatteryBankTemperatureUpdater(x_begin_loc + 20, y_begin_loc + 130, bank_temperature_array[1]);
  BatteryBankTemperatureUpdater(x_begin_loc + 20, y_begin_loc + 240, bank_temperature_array[2]);
  BatteryBankTemperatureUpdater(x_begin_loc + 20, y_begin_loc + 350, bank_temperature_array[3]);
  BatteryBankTemperatureUpdater(x_begin_loc + 20, y_begin_loc + 460, bank_temperature_array[4]);
  BatteryBankTemperatureUpdater(x_begin_loc + 20, y_begin_loc + 570, bank_temperature_array[5]);

  BatteryBankTemperatureUpdater(x_begin_loc + 210, y_begin_loc + 20, bank_temperature_array[6]);
  BatteryBankTemperatureUpdater(x_begin_loc + 210, y_begin_loc + 130, bank_temperature_array[7]);
  BatteryBankTemperatureUpdater(x_begin_loc + 210, y_begin_loc + 240, bank_temperature_array[8]);
  BatteryBankTemperatureUpdater(x_begin_loc + 210, y_begin_loc + 350, bank_temperature_array[9]);
  BatteryBankTemperatureUpdater(x_begin_loc + 210, y_begin_loc + 460, bank_temperature_array[10]);
  BatteryBankTemperatureUpdater(x_begin_loc + 210, y_begin_loc + 570, bank_temperature_array[11]);
  
  BatteryBankTemperatureUpdater(x_begin_loc + 410, y_begin_loc + 20, bank_temperature_array[12]);    
  BatteryBankTemperatureUpdater(x_begin_loc + 410, y_begin_loc + 130, bank_temperature_array[13]);    
  BatteryBankTemperatureUpdater(x_begin_loc + 410, y_begin_loc + 240, bank_temperature_array[14]);   
  BatteryBankTemperatureUpdater(x_begin_loc + 410, y_begin_loc + 350, bank_temperature_array[15]);   
  BatteryBankTemperatureUpdater(x_begin_loc + 410, y_begin_loc + 460, bank_temperature_array[16]);   
  BatteryBankTemperatureUpdater(x_begin_loc + 410, y_begin_loc + 570, bank_temperature_array[17]);  

}

void voltageRangeChecker(){
  
  if (OPEN_WIRE_ARRAY[0] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[0] = color(255);
  }
  else{
  if (bank_temperature_array[0] > TEMP_THRESHOLD){bank_color_array[0] = color(255, 50, 0);}
  else{
  if (bank_voltages[0] == 0){bank_color_array[0] = color(255);}
  else if (bank_voltages[0] <= MINIMUM_VOLTAGE && bank_voltages[0] > 0.0){bank_color_array[0] = color(255, 255, 0);}
  else if (bank_voltages[0] > MINIMUM_VOLTAGE && bank_voltages[0] <= MAX_VOLTAGE){bank_color_array[0] = color(0, 255, 0);}
  else  if (bank_voltages[0] > MAX_VOLTAGE){bank_color_array[0] = color(255, 192, 192);}
  else{bank_color_array[0] = color(255);}}}
  
  if (OPEN_WIRE_ARRAY[1] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[1] = color(255);
  }
  else{
  if (bank_temperature_array[1] > TEMP_THRESHOLD){bank_color_array[1] = color(255, 50, 0);}
  else{
  if (bank_voltages[1] == 0){bank_color_array[1] = color(255);}
  else if (bank_voltages[1] <= MINIMUM_VOLTAGE && bank_voltages[1] > 0.0){bank_color_array[1] = color(255, 255, 0);}
  else if (bank_voltages[1] > MINIMUM_VOLTAGE && bank_voltages[1] <= MAX_VOLTAGE){bank_color_array[1] = color(0, 255, 0);}
  else  if (bank_voltages[1] > MAX_VOLTAGE){bank_color_array[1] = color(255, 192, 192);}
  else{bank_color_array[1] = color(255);}}}

  if (OPEN_WIRE_ARRAY[2] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[2] = color(255);
  }
  else{
  if (bank_temperature_array[2] > TEMP_THRESHOLD){bank_color_array[2] = color(255, 50, 0);}
  else{
  if (bank_voltages[2] == 0){bank_color_array[2] = color(255);}
  else if (bank_voltages[2] <= MINIMUM_VOLTAGE && bank_voltages[2] > 0.0){bank_color_array[2] = color(255, 255, 0);}
  else if (bank_voltages[2] > MINIMUM_VOLTAGE && bank_voltages[2] <= MAX_VOLTAGE){bank_color_array[2] = color(0, 255, 0);}
  else  if (bank_voltages[2] > MAX_VOLTAGE){bank_color_array[2] = color(255, 192, 192);}
  else{bank_color_array[2] = color(255);}}}

  if (OPEN_WIRE_ARRAY[3] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[3] = color(255);
  }
  else{
  if (bank_temperature_array[3] > TEMP_THRESHOLD){bank_color_array[3] = color(255, 50, 0);}
  else{  
  if (bank_voltages[3] == 0){bank_color_array[3] = color(255);}
  else if (bank_voltages[3] <= MINIMUM_VOLTAGE && bank_voltages[3] > 0.0){bank_color_array[3] = color(255, 255, 0);}
  else if (bank_voltages[3] > MINIMUM_VOLTAGE && bank_voltages[3] <= MAX_VOLTAGE){bank_color_array[3] = color(0, 255, 0);}
  else  if (bank_voltages[3] > MAX_VOLTAGE){bank_color_array[3] = color(255, 192, 192);}
  else{bank_color_array[3] = color(255);}}}

  if (OPEN_WIRE_ARRAY[4] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[4] = color(255);
  }
  else{
  if (bank_temperature_array[4] > TEMP_THRESHOLD){bank_color_array[4] = color(255, 50, 0);}
  else{  
  if (bank_voltages[4] == 0){bank_color_array[4] = color(255);}
  else if (bank_voltages[4] <= MINIMUM_VOLTAGE && bank_voltages[4] > 0.0){bank_color_array[4] = color(255, 255, 0);}
  else if (bank_voltages[4] > MINIMUM_VOLTAGE && bank_voltages[4] <= MAX_VOLTAGE){bank_color_array[4] = color(0, 255, 0);}
  else  if (bank_voltages[4] > MAX_VOLTAGE){bank_color_array[4] = color(255, 192, 192);}
  else{bank_color_array[4] = color(255);}}}

  if (OPEN_WIRE_ARRAY[5] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[5] = color(255);
  }
  else{
  if (bank_temperature_array[5] > TEMP_THRESHOLD){bank_color_array[5] = color(255, 50, 0);}
  else{  
  if (bank_voltages[5] == 0){bank_color_array[5] = color(255);}
  else if (bank_voltages[5] <= MINIMUM_VOLTAGE && bank_voltages[5] > 0.0){bank_color_array[5] = color(255, 255, 0);}
  else if (bank_voltages[5] > MINIMUM_VOLTAGE && bank_voltages[5] <= MAX_VOLTAGE){bank_color_array[5] = color(0, 255, 0);}
  else  if (bank_voltages[5] > MAX_VOLTAGE){bank_color_array[5] = color(255, 192, 192);}
  else{bank_color_array[5] = color(255);}}}

  if (OPEN_WIRE_ARRAY[6] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[6] = color(255);
  }
  else{
  if (bank_temperature_array[6] > TEMP_THRESHOLD){bank_color_array[6] = color(255, 50, 0);}
  else{  
  if (bank_voltages[6] == 0){bank_color_array[6] = color(255);}
  else if (bank_voltages[6] <= MINIMUM_VOLTAGE && bank_voltages[6] > 0.0){bank_color_array[6] = color(255, 255, 0);}
  else if (bank_voltages[6] > MINIMUM_VOLTAGE && bank_voltages[6] <= MAX_VOLTAGE){bank_color_array[6] = color(0, 255, 0);}
  else  if (bank_voltages[6] > MAX_VOLTAGE){bank_color_array[6] = color(255, 192, 192);}
  else{bank_color_array[6] = color(255);}}}

  if (OPEN_WIRE_ARRAY[7] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[7] = color(255);
  }
  else{
  if (bank_temperature_array[7] > TEMP_THRESHOLD){bank_color_array[7] = color(255, 50, 0);}
  else{  
  if (bank_voltages[7] == 0){bank_color_array[7] = color(255);}
  else if (bank_voltages[7] <= MINIMUM_VOLTAGE && bank_voltages[7] > 0.0){bank_color_array[7] = color(255, 255, 0);}
  else if (bank_voltages[7] > MINIMUM_VOLTAGE && bank_voltages[7] <= MAX_VOLTAGE){bank_color_array[7] = color(0, 255, 0);}
  else  if (bank_voltages[7] > MAX_VOLTAGE){bank_color_array[7] = color(255, 192, 192);}
  else{bank_color_array[7] = color(255);}}}

  if (OPEN_WIRE_ARRAY[8] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[8] = color(255);
  }
  else{
  if (bank_temperature_array[8] > TEMP_THRESHOLD){bank_color_array[8] = color(255, 50, 0);}
  else{  
  if (bank_voltages[8] == 0){bank_color_array[8] = color(255);}
  else if (bank_voltages[8] <= MINIMUM_VOLTAGE && bank_voltages[8] > 0.0){bank_color_array[8] = color(255, 255, 0);}
  else if (bank_voltages[8] > MINIMUM_VOLTAGE && bank_voltages[8] <= MAX_VOLTAGE){bank_color_array[8] = color(0, 255, 0);}
  else  if (bank_voltages[8] > MAX_VOLTAGE){bank_color_array[8] = color(255, 192, 192);}
  else{bank_color_array[8] = color(255);}}}

  if (OPEN_WIRE_ARRAY[9] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[9] = color(255);
  }
  else{
  if (bank_temperature_array[9] > TEMP_THRESHOLD){bank_color_array[9] = color(255, 50, 0);}
  else{  
  if (bank_voltages[9] == 0){bank_color_array[9] = color(255);}
  else if (bank_voltages[9] <= MINIMUM_VOLTAGE && bank_voltages[9] > 0.0){bank_color_array[9] = color(255, 255, 0);}
  else if (bank_voltages[9] > MINIMUM_VOLTAGE && bank_voltages[9] <= MAX_VOLTAGE){bank_color_array[9] = color(0, 255, 0);}
  else  if (bank_voltages[9] > MAX_VOLTAGE){bank_color_array[9] = color(255, 192, 192);}
  else{bank_color_array[9] = color(255);}}}

  if (OPEN_WIRE_ARRAY[10] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[10] = color(255);
  }
  else{
  if (bank_temperature_array[10] > TEMP_THRESHOLD){bank_color_array[10] = color(255, 50, 0);}
  else{  
  if (bank_voltages[10] == 0){bank_color_array[10] = color(255);}
  else if (bank_voltages[10] <= MINIMUM_VOLTAGE && bank_voltages[10] > 0.0){bank_color_array[10] = color(255, 255, 0);}
  else if (bank_voltages[10] > MINIMUM_VOLTAGE && bank_voltages[10] <= MAX_VOLTAGE){bank_color_array[10] = color(0, 255, 0);}
  else  if (bank_voltages[10] > MAX_VOLTAGE){bank_color_array[10] = color(255, 192, 192);}
  else{bank_color_array[10] = color(255);}}}

  if (OPEN_WIRE_ARRAY[11] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[11] = color(255);
  }
  else{
  if (bank_temperature_array[11] > TEMP_THRESHOLD){bank_color_array[11] = color(255, 50, 0);}
  else{
  if (bank_voltages[11] == 0){bank_color_array[11] = color(255);}
  else if (bank_voltages[11] <= MINIMUM_VOLTAGE && bank_voltages[11] > 0.0){bank_color_array[11] = color(255, 255, 0);}
  else if (bank_voltages[11] > MINIMUM_VOLTAGE && bank_voltages[11] <= MAX_VOLTAGE){bank_color_array[11] = color(0, 255, 0);}
  else  if (bank_voltages[11] > MAX_VOLTAGE){bank_color_array[11] = color(255, 192, 192);}
  else{bank_color_array[11] = color(255);}}}

  if (OPEN_WIRE_ARRAY[12] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[12] = color(255);
  }
  else{
  if (bank_temperature_array[12] > TEMP_THRESHOLD){bank_color_array[12] = color(255, 50, 0);}
  else{
  if (bank_voltages[12] == 0){bank_color_array[12] = color(255);}
  else if (bank_voltages[12] <= MINIMUM_VOLTAGE && bank_voltages[12] > 0.0){bank_color_array[12] = color(255, 255, 0);}
  else if (bank_voltages[12] > MINIMUM_VOLTAGE && bank_voltages[12] <= MAX_VOLTAGE){bank_color_array[12] = color(0, 255, 0);}
  else  if (bank_voltages[12] > MAX_VOLTAGE){bank_color_array[12] = color(255, 192, 192);}
  else{bank_color_array[13] = color(255);}}}

  if (OPEN_WIRE_ARRAY[13] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[13] = color(255);
  }
  else{
  if (bank_temperature_array[13] > TEMP_THRESHOLD){bank_color_array[13] = color(255, 50, 0);}
  else{  
  if (bank_voltages[13] == 0){bank_color_array[13] = color(255);}
  else if (bank_voltages[13] <= MINIMUM_VOLTAGE && bank_voltages[13] > 0.0){bank_color_array[13] = color(255, 255, 0);}
  else if (bank_voltages[13] > MINIMUM_VOLTAGE && bank_voltages[13] <= MAX_VOLTAGE){bank_color_array[13]= color(0, 255, 0);}
  else  if (bank_voltages[13] > MAX_VOLTAGE){bank_color_array[13] = color(255, 192, 192);}
  else{bank_color_array[13] = color(255);}}}

  if (OPEN_WIRE_ARRAY[14] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[14] = color(255);
  }
  else{
  if (bank_temperature_array[14] > TEMP_THRESHOLD){bank_color_array[14] = color(255, 50, 0);}
  else{  
  if (bank_voltages[14] == 0){bank_color_array[12] = color(255);}
  else if (bank_voltages[14] <= MINIMUM_VOLTAGE && bank_voltages[14] > 0.0){bank_color_array[14] = color(255, 255, 0);}
  else if (bank_voltages[14] > MINIMUM_VOLTAGE && bank_voltages[14] <= MAX_VOLTAGE){bank_color_array[14] = color(0, 255, 0);}
  else  if (bank_voltages[14] > MAX_VOLTAGE){bank_color_array[14] = color(255, 192, 192);}
  else{bank_color_array[14] = color(255);}}}

  if (OPEN_WIRE_ARRAY[15] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[15] = color(255);
  }
  else{
  if (bank_temperature_array[15] > TEMP_THRESHOLD){bank_color_array[15] = color(255, 50, 0);}
  else{
  if (bank_voltages[15] == 0){bank_color_array[12] = color(255);}
  else if (bank_voltages[15] <= MINIMUM_VOLTAGE && bank_voltages[15] > 0.0){bank_color_array[15] = color(255, 255, 0);}
  else if (bank_voltages[15] > MINIMUM_VOLTAGE && bank_voltages[15] <= MAX_VOLTAGE){bank_color_array[15] = color(0, 255, 0);}
  else  if (bank_voltages[15] > MAX_VOLTAGE){bank_color_array[15] = color(255, 192, 192);}
  else{bank_color_array[15] = color(255);}}}

  if (OPEN_WIRE_ARRAY[16] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[16] = color(255);
  }
  else{
  if (bank_temperature_array[16] > TEMP_THRESHOLD){bank_color_array[16] = color(255, 50, 0);}
  else{  
  if (bank_voltages[16] == 0){bank_color_array[16] = color(255);}
  else if (bank_voltages[16] <= MINIMUM_VOLTAGE && bank_voltages[16] > 0.0){bank_color_array[16] = color(255, 255, 0);}
  else if (bank_voltages[16] > MINIMUM_VOLTAGE && bank_voltages[16] <= MAX_VOLTAGE){bank_color_array[16] = color(0, 255, 0);}
  else  if (bank_voltages[16] > MAX_VOLTAGE){bank_color_array[16] = color(255, 192, 192);}
  else{bank_color_array[16] = color(255);}}}

  if (OPEN_WIRE_ARRAY[17] == true){  //Checks if there is an open wire on this Bank.
     bank_color_array[17] = color(255);
  }
  else{
  if (bank_temperature_array[17] > TEMP_THRESHOLD){bank_color_array[17] = color(255, 50, 0);}
  else{
  if (bank_voltages[17] == 0){bank_color_array[12] = color(255);}
  else if (bank_voltages[17] <= MINIMUM_VOLTAGE && bank_voltages[17] > 0.0){bank_color_array[17] = color(255, 255, 0);}
  else if (bank_voltages[17] > MINIMUM_VOLTAGE && bank_voltages[17] <= MAX_VOLTAGE){bank_color_array[17] = color(0, 255, 0);}
  else  if (bank_voltages[17] > MAX_VOLTAGE){bank_color_array[17] = color(255, 192, 192);}
  else{bank_color_array[17] = color(255);}}}



}


void BatteryControlBox(int x, int y, String text){

  stroke(180);
  strokeWeight(1);
  noFill();
  if (text == "Max Diff"){
      line(x, y, x + 5, y);
      line(x + 55, y, x + 175, y);
      line(x, y, x, y + 45);
      line(x, y + 45, x + 175, y + 45);
      line(x + 175, y + 45, x + 175, y);
      fill(0);
      textSize(10);
      text(text, x + 10, y + 3);
      fill(255);
      rect(x + 15, y + 10, 150, 30);
  }

  else{
      line(x, y, x + 5, y);
      line(x + 55, y, x + 175, y);
      line(x, y, x, y + 20);
      line(x, y + 20, x + 175, y + 20);
      line(x + 175, y + 20, x + 175, y);
      fill(0);
      textSize(10);
      text(text, x + 10, y + 3);
  }
  fill(0);
}

//Battery Banks in Mointroing widget
void BatteryBankFigure(int x, int y, String text){
  //Widget Lines in Cell banks.
  noFill();
  textSize(10);
  line(x, y, x + 10, y);
  line(x + 72, y, x + 175, y);
  line(x, y, x, y + 100);
  line(x, y + 100, x + 175, y + 100);
  line(x + 175, y + 100, x + 175, y);
  fill(255);
  //Voltage Values white boxes.
  rect(x+85, y+55, 85, 40);
  fill(0);
  textSize(13);
  text(text, x + 15, y + 5);
  //Temperature Values white Box.
  fill(255);
  rect(x+85, y+15, 85, 35);
}



void BoxInsideStatus(int x, int y, String textInBox){
  
  if (textInBox == "CHARGE"|| textInBox == "CURRENT" || textInBox == "MAX TEMP"){
    line(x +20, y+35, x + 40, y+35);
    line(x + 110, y+35, x + 275, y+35);
    line(x+20, y+35, x+20,y + 250);
    line(x+20, y + 250, x + 275, y + 250);
    line(x + 275, y + 250, x + 275, y+35);
    
    if (textInBox == "MAX TEMP"){fill(0);text(textInBox, x+42.5, y+40);}
  
    else{fill(0);text(textInBox, x+50, y+40);}
  }  
  if (textInBox == "ENERGY" || textInBox == "POWER"){
    line(x +20, y+35, x + 40, y+35);
    line(x + 110, y+35, x + 275, y+35);
    line(x+20, y+35, x+20,y + 185);
    line(x+20, y + 185, x + 275, y + 185);
    line(x + 275, y + 185, x + 275, y+35);
    fill(0);
    text(textInBox, x+50, y+40);} 
    
  if (textInBox == "CHARGE"){ 
    fill(255);
    rect(x+ 32.5, y +210, 110, 30);
    rect(x+ 152.5, y +210, 110, 30);}
    
  if (textInBox == "INDICATORS"){
    line(x +20, y+35, x + 40, y+35);
    line(x + 130, y+35, x + 275, y+35);
    line(x+20, y+35, x+20,y + 250);
    line(x+20, y + 250, x + 275, y + 250);
    line(x + 275, y + 250, x + 275, y+35);
    fill(0);
    text(textInBox, x+50, y+40);
  //  text("Faults", x+115, y+70);
  }

}

//Boxes in DC2732 window.
void BoxInsideDC2732Diagonsis(int x, int y, String textInBox){
  
  if (textInBox == "BOARD TEMP"){
    line(x +20, y+30, x + 35, y+30);
    line(x + 125, y+30, x + 150, y+30);
    line(x+20, y+30, x+20,y + 150);
    line(x+20, y + 150, x + 150, y + 150);
    line(x + 150, y + 150, x + 150, y+30);  
    fill(0);
    textSize(13);  
    text(textInBox, x+40, y+35);
 }
  else if (textInBox == "INTER TEMP"){ 
    line(x +20, y+30, x + 35, y+30);
    line(x + 125, y+30, x + 150, y+30);
    line(x+20, y+30, x+20,y + 150);
    line(x+20, y + 150, x + 150, y + 150);
    line(x + 150, y + 150, x + 150, y+30);  
    fill(0);
    textSize(13);  
    text(textInBox, x+45, y+35);
  }
}

//Boxes in DC2350B window
void BoxInsideDC2350Diagonsis(int x, int y, String textInBox){
  
  if (textInBox == "TEMP"){
    line(x +20, y+30, x + 40, y+30);
    line(x + 90, y+30, x + 150, y+30);
    line(x+20, y+30, x+20,y + 150);
    line(x+20, y + 150, x + 150, y + 150);
    line(x + 150, y + 150, x + 150, y+30);  
    fill(0);
    textSize(13);  
    text(textInBox, x+50, y+35);
 }
  else{ 
    line(x +20, y+35, x + 30, y+35);
    line(x + 80, y+35, x + 180, y+35);
    line(x+20, y+35, x+20,y + 65);
    line(x+20, y + 65, x + 180, y + 65);
    line(x + 180, y + 65, x + 180, y+35); 
    fill(255);
    //white boxes inside each box.
    rect(x+25, y+40, 150, 20);
    
    fill(0);
    textSize(13);  
    text(textInBox, x+40, y+35);}
}

void batteryFigure(int xPos, int yPos, int Width, int Height){
  fill(255);stroke(0);strokeWeight(2);strokeCap(ROUND);
  
  //Draw Top of Battery.
  beginShape();
  vertex(xPos+Width/4, yPos);
  vertex(xPos+Width/4, yPos-Height/5);
  vertex(xPos+3*Width/4, yPos-Height/5);
  vertex(xPos+3*Width/4, yPos);
  vertex(xPos+Width/4, yPos);
  endShape(CLOSE);
  
  //Draw Battery Rectangle. 
  beginShape();
  vertex(xPos, yPos);
  vertex(xPos+Width, yPos);
  vertex(xPos+Width, yPos+Height);
  vertex(xPos, yPos+Height);
  endShape(CLOSE);

}







//Figures for the legend Window.
void BatteryLegendFigures(int xPos, int yPos, int Width, int Height, color Color, String Text, String Text1){ 
  fill(0);stroke(0);strokeWeight(2);strokeCap(ROUND);
  rect(xPos-20, yPos-40, Width+40, Height/2);
  textSize(12);
  text(Text1, xPos-30, yPos+60);  
  fill(Color);
  textSize(13);
  text(Text, xPos, yPos-25);
  beginShape();
  vertex(xPos, yPos);
  vertex(xPos+Width/4, yPos);
  vertex(xPos+Width/4, yPos-Height/5);
  vertex(xPos+3*Width/4, yPos-Height/5);
  vertex(xPos+3*Width/4, yPos);
  vertex(xPos+Width/4, yPos);
  vertex(xPos+Width, yPos);
  vertex(xPos+Width, yPos+Height);
  vertex(xPos, yPos+Height);
  endShape(CLOSE);
}

void exit(){
      SerialOutput.flush();
      SerialOutput.close();
      super.exit();
}
