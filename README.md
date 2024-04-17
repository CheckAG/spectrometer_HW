### Test Stub: 1
**Arduino to PC data transfer**
a) Setup Arduino to fastest possible data rate  
b) Create a known pattern (Sin look up table or similar predictable data)  
c) Transfer data from Arduino to PC over serial port for a specific duration  
d) Store the data in a file on PC  
e) Write a script to validate the data  
f) Automate the test rig and validate the performance and characterize the maximum throughput achievable  

### Test Stub: 2
**Serial Port: DMA transfer**
a) In Test Stub 1, it is possible to use the MCU to dedicate the CPU cycles only for data transfer  
b) However, the CPU will eventually have to be freed up for other tasks like buffer management, interrupt management, etc  
c) We will have to use DMA to transfer data to the serial port and also implement triple buffering to automate data movement from Arduino to PC  
d) There is an event manager in SAMD21, which needs to be used to trigger the DMA transfer  
e) The test stub should facilitate data transfer to PC using DMA in Arduino  
f) Once the measurement is completed, we can characterize the data throughput realized using Arduino sending data employing DMA  

### Test Stub: 3
**ADC throughput**
a) The theoretical data throughput of the ADC is 350 ksps, but it will be difficult to achieve the same in practical conditions  
b) We have to ensure that the digital conversion is error-free and is synchronized with the sensor clocks  
c) Using a DAC or a signal generator, we can create known signals and using the ADC, we can capture and send to PC for further analysis  
d) A test stub featuring the ADC throughput validation will ensure that the ADC is working in synchronization with the sensor clocks  
