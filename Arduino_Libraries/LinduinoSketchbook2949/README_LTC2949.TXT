Linduino Sketches for LTC2949:

\Part Number\2000\2900\2949\...
	DC2732A_SM     – All safety measures
	DC2732A_FIFO12 – Fast I1+I2, reading fifo samples and slow precision results
	DC2732A_EXT    – The most complete, but also most complex sketch, supporting all configurations / measurements
	DC2732A_CAN    – For the can based evaluation system (needs DC2617A)
	DC2732A_BASIC  - basic sketch showing continuous slow and fast measurements
	DC2732A_ADV    - advanced sketch showing more complex measurement cycles with different update rates and fast single shot round-robin
	DC2732A_681X   - basic sketch showing synchronous measurements of current with LTC2949 and cell voltages with LTC6811 (or other compatible devices of the LTC68xx family)
	DC2732A_TEST   - sketch used for DC2732A production test. Shows usage of GPOs, fast measurements...
  
\Part Number\2000\2900\2949\DC2732A_TINY\...: Tiny Linduino Sketches for LTC2949
	DC2732A_NTC         – NTC measurement
	DC2732A_FASTSTARTUP – Demonstrates the fastest way to start-up LTC2949 and make a measurement
	DC2732A_BCI         – Example code to be used for BCI, EFT or other communication noise immunity tests
	DC2732A_FSSHT_RR    – Example code to show fast AUX round-robin

\Utilities\MUNDUINO\MunduinoBinIF\
	MunBinIF2949 - Serial port binary communication interface to LTC2949 used for LTC2949 GUI (OPTIONAL! The GUI also supports the Linduino's default DC590B sketch)
	

They all use the same library files LTC2949.h and LTC2949.cpp which are placed in
\libraries\LTC2949


Please extract into the same folder were the official ADI / LTC Linduino sketchbook is extracted to.

The official ADI / LTC Linduino sketchbook is available here:
https://github.com/analogdevicesinc/Linduino

patrick.wilhelm@analog.com


