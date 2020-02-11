# Serial Communication
> This is a note for the following article(s)
> https://learn.sparkfun.com/tutorials/serial-communication/wiring-and-hardware


## What is TTL (Transistor-transistor logic) serial communication
TTL serial communication means the serial data is usually transferred at logic level and
the voltage is from 0~3.3V/5V. A signal at VCC level (3.3V, 5V, etc) indicates an idle line, 
a stop bit or a bit value of 1, while a signal at 0V (GND) indicates either a start bit or a bit
value of 0.

<figure>
    <img src="https://cdn.sparkfun.com/assets/1/8/d/c/1/51142c09ce395f0e7e000002.png" alt="TTL"/>
    <figcaption>TTL serial communication</figcaption>
</figure>
