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


## What is UART (Universal Asynchronous Receiver/Transmitter)
UART is a block of **circuitry** responsible for implementing serial communication. It's not a protocal but a hardware instead. It's a module commonly embedded in a microcontroller. Some microcontroller has none, some may have one or more. It's working like an intermediate component between parallel and serial interface.
<figure>
    <img src="https://cdn.sparkfun.com/assets/d/1/f/5/b/50e1cf30ce395fb227000000.png" />
    <figcaption>Super-simplified UART interface. Parallel on one end, serial on the other</figcaption>
</figure>