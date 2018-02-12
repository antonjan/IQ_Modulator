# IQ_Modulator
IQ Modulator for 2M and 70cm form I2C PCM audio codec for modelation (estimated 500mW)<br>
I2C Control for LO Local Osxelator with IQ moualtor RF amp and SAW and Ceremic output filters to antenna.<br>
<b><This is not complete yet still designing ></b>
I am considering using Orange Pi rather than Raspberry as it has build in Audio in and out and footprint is smaller<br>
I nother atvantage of the Orange pi is the additional USB ports witch I want to enable for the RTL dongle to be used as receiver.<br>
The software is compleet using gnuradio for transponder using hackrf for TX and rtl sdr for RX.<br>
I am still working on the telemetry witch works on its own but i need to combine it with Gnuraio transponder.<br>
I have to telemetry modes at the moment CW send Calsign and cubesat name and settings. APRS 1200fsk sending fukk status of transponder<br>  
The design was done in Kicad<br>
<br>
todo<br>
1) Real time clock (done)<br>
2) I2C pull up resistors (done)<br>
3) Maybe different 70cm filter
4) CS for ic's<br>
5) Temprature and Tx power measurement (A/D)
6) Battery Voltage Monitoring (A/D)


