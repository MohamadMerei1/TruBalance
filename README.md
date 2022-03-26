# TruBalance BMS


This repository contains the Arduino and GUI codes used in the TurBalance BMS.

<p align="Center">
<img src="images/logo.JPG">
</p>

# Arduino


<p align="center">
<img src="images/arduino.png" width="150" height="100">
</p>

The user of this system must confirm the hardware connection before uploading any code to the Linduino board. The hardware connections from linduino to the other boards must be confirmed with the schmatic design provided above, pictures of the current colored wires (utilized by the TruBalance Team) are also provided. 

- Arduino Setup:
   
      - [Download and install the Arduino IDE (Recommended Version 1.8 or Later).](https://www.arduino.cc/en/software) 
      - [Download the Arduino Libraries from the folder above.](https://github.com/MohamadMerei1/TruBalance/tree/main/Arduino_Libraries)
      - Unzip the LinduinoSketch folder.
      - Locate the Arduino libraries folder:
      `C:\Users\moham\Documents\Arduino\`
      - Place the downloaded libraries inside the Arduino folder from the path above, where  
        the folder will look like the following:
        <p align="right">
        <img src="images/Untitled drawing.png">
        </p>
      - Place the `ltcmuc_tools` and `ltcmuc_tools_ext` folders from: 
      `~\Arduino\LinduinoSketchbook2949\libraries\`, in:  
      `~\Arduino\LTSketchbook\libraries`
      - Start the Arduino IDE
      - Go into Preferences under File
      - Change the sketch location to: `C:\Users\moham\Documents\Arduino\LTSketchbook`, adjust this based on the location in your PC. 
        <p align="right">
        <img src="images/Capture1.PNG">
        </p>
      - Other libraries may be required to be installed, you can always do so by locating the Library Manager Button under tools and searching for the needed library (This is unlikely).
      - Once all libraries are installed, Restart the Arduino IDE
      - Locate Board button under Tools and choose Arduino UNO
      - Make sure your USB is connected to a Linduino 
      - Choose the correct port under Tools
      - Download the sketch from Arduino Code folder above and Upload it to the Linduino Board 
      - If no errors appear in the console window, this would mean the linduino board was programmed successfully
      - Some steps to Troubleshooting errors:  
      
            1. Restart Arduino IDE
            2. Make sure libraries are located as explained above
            3. Your OS may not have the correct USB driver, search online for the correct one
            4. Try a difference OS
        
If these steps were followed carfully the linduino board should now contain the program needed for runnning the system. 

# Processing

<p align="center">
<img src="images/processing.png" width="200height="150
</p>


# Credit

The arduino code and libraries utilized in this project are based on a modified version
of the sketches provided by Analog Devcies. 

<p align="center">
  <img src="images/analogDevices.png">
</p>
