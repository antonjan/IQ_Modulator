Deails here http://www.radiotecnia.es/raspberry-pi-based-psk31-transmitter/

The service must be run with root permissions. Help is shown with this command:

pi@raspberrypi ~/psk31 $ ./psk31 --help
Options:
 --amplitude=<n> Signal amplitude (0 .. 1]
 --clock-div=<n> Fractional divisor for carrier [4096 .. 16773120]
 Note: frequency = 500 MHz / (clock-div / 4096)
 --frequency=<f> Carrier frequency, in MHz [0.125 .. 500]
 Note: this is overridden by clock-div
 --help Show this help
 --mash=<n> Set number of MASH stages [0 .. 3]
 --pcm Use PCM clock instead of PWM clock for signal generation
 --rc=<f> Set signal filter RC value (s)
 --timeout=<n> Number of zeros before switching off. 0 for infinite.

Amplitude should be the highest. “mash” parameter should be set to ‘1’. A ‘0’ value disables fractional pll values above 25 MHz. “rc” time constant should be that of the filter we have built. If we run the program with these parameters, an unmodulated carrier is sent:

    sudo ./psk31 --mash=1 --rc=0.0047 --timeout=20 --frequency=7.042

No root permissions are needed for sending text. We can dump the content on the transmission buffer through /dev/psk31.data device as shown:

    echo "CQ CQ" > /dev/psk31.data

    cat fichero.txt > /dev/psk31.data

In this way:

    cat > /dev/psk31.data

we enter an interactive mode. We can write in the console and <Enter> sends the line to the buffer.

We can run this command to know the state of the service:

    nc -U /dev/psk31.stat

Which returns:

    amplitude 0.900000
    rc 0.004700
    clock_div 290826
    clock_mash 1
    clock_freq 7.042011
    timeout 20
    pending_char 53

To stop the service:

    sudo killall psk31

The actual divider value is not the “clock_div” number. The pll has a 500 MHz reference which is divided by a number with integer and fractional parts each represented with 12 bits. That is to say, it can divide fractions 2^12 or 4096 times smaller than one. In this case, 290826 means 500 is divided by 71 + 10/4096 (as 71·4096=290816). We have launched the service for 7.042 MHz which is obtained as 500 · 4096 / 290826 = 7.042.

This also means the resolution (the frequency step) is not fixed, but dependent on the starting frequency. Being ‘N’ an integer number between 2^13 and 2^23 that is 8.192 and 8.388.608, by means of this equation:

F = 500e6/(N/4096) [Hz]

we get frequencies between 244 kHz and 250 MHz. The rest of dividing N by 4096 (2¹²) is the fractional part.

The resolution is:

f1-f2 = 500e6/(N/4096) – 500e6/((N+1)/4096))

After some algebra:

f1 – f2 = f1² / (500e6*4096) [Hz]

Which means the resolution step is proportional to the square of the frequency.

For example, in 145 MHz  10 kHz.
In 50 MHz the step is 1,2 kHz.
In 28 MHz the step is 382 Hz.
In 7 MHz the step is 24 Hz.

This video shows the first trials of the prototype, without amplification, transmitting a short text across the room:

Conclusions and future prospects

My signal in 20 and 40 meters bands was heard 5 km away. This circuit can be improved regarding the schematic, pcb design and component election, but its goal was proving the feasibility of the idea. It is not supposed to be a convinient way to make QSOs because, among other things, a separate receiver and decoder would be needed. But it could be useful as an autonomous beacon for telemetry or propagation study. Moreover, the sigma-delta and the baseband coupling are tools which can be utilized in different projects.

The circuit could be improved in the following ways: pin 16 (gpio 23) is set at high level when the service is running. It could be used for biasing the base of 2N2222 transistor and prevent it from drawing current when unnecessary. The oscillator has a square waveshape and a dedicate filter for each band should be used. Likewise, SA602 differential output would be better coupled by means of a transformer in order to reduce distorsion and duplicate power. The same circuit can be used with the programs indicated above for transmitting other modulations, save for rpitx which makes use of pin 12 (gpio 18) for the PLL output instead of pin 7 (gpio 4); the connections should be made interchangeable.

By EA4GMZ • Electrónica y equipos, Otros • 
