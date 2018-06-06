# IQ_Modulator for Cubesat Transmitter.
![Alt text](doc/IQ_Modulator_Digram_1.png?raw=true "PCB")<br>
IQ Modulator for 2M generating IQ stream via Audio PCM codec as modelation and using I2C to controle the RF local oselator for the RF mixer. The modulated RF signal is the Amplified to about +27dbm. (estimated 500mW)<br>
I2C Control for LO Local Osxelator with IQ moualtor RF amp and SAW and Ceremic output filters to antenna.<br>
<b><This is not complete yet still designing ></b>
![Alt text](doc/IQ_Modulator_PCB_1.png?raw=true "PCB")<br>
# OBC and SDR  
I am considering using Orange Pi rather than Raspberry as it has build in Audio in and out with a smaller footprint<br>
An another advantage of the Orange Pi is the additional USB ports witch I want to enable for the RTL dongle to be used as receiver.<br>
The software concept was tested using gnuradio for transponder with a hackrf for TX and a rtl sdr dongle for RX.<br>
I am still working on the telemetry witch works on its own but I need to combine it with Gnuraio transponder.<br>
I have to telemetry modes at the moment CW send Calsign and cubesat name and settings. APRS 1200 FSK sending the status of transponder<br>
# Power
Total power usage of board still needs to be calculated but my estimation with full power is 2w including Orange pi.
8)The sleep mode possibly 500mw<br>
The PCB design was done in Kicad<br>
<br>
# Todo
1) Real time clock (done) I had to remove the Real time clock as there is not enuf space on the PCB<br>
2) I2C pull up resistors (done)<br>
3) Maybe different 70cm filter. (2m filter was changed to discrete components pi 5 stage 40db down. The 70cm filter was removed as there was now space on the PCB)<br>
4) CS for ic's<br>
5) Temprature and Tx power measurementA/D. (The A/D was added using MCP3421A0T-E/CH and will read the power output of the MMZ09332BT1 30db power amp that should give us about +27dbm. This A/D reading can then be used to adjust the ALC level in the IQ stream)<br>
6) Battery and solar Voltage Monitoring (A/D) (I had to remove this as there was no space on the PCB Board)<br>

  


