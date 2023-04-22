Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.* 

1. What is the average current per period?
   Answer: 21.49 uA
   <br>Screenshot:  
   ![Avg_current_per_period](/Screenshots/assignment_4/Avg_current_per_period.jpg)  

2. What is the average current when the Si7021 is Powered Off?
   Answer: 2.31 uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](/Screenshots/assignment_4/Avg_current_LPM_Off.jpg)  

3. What is the average current when the Si7021 is Powered On?
   Answer: 529.17 uA
   <br>Screenshot:  
   ![Avg_current_LPM_On](/Screenshots/assignment_4/Avg_current_LPM_On.jpg)  

4. How long is the Si7021 Powered On for 1 temperature reading?
   Answer: 100.80 ms
   <br>Screenshot:  
   ![duration_lpm_on](/Screenshots/assignment_4/duration_lpm_on.jpg)  

5. Compute what the total operating time of your design for assignment 4 would be in hours, assuming a 1000mAh battery power supply?
   Answer: Sensor consumes 21.49 uA every 3 seconds. Hence in 1 hr it would consume 25.788 mA. Thus total operating time = 1000/25.788 = 38.77 hrs.
   
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer: The power consumption has significantly reduced from the previous assignment. In the previous assignment we used a polling approach for both I2C transactions and the timer delays. In this assignment we have shifted to an interrupt driven approach. This allows the microcontroller to sleep after starting the transaction/timer. This saves power since the current consumption during this time is very low. 
   
7. Describe how you tested your code for EM1 during I2C transfers.
   Answer: The energy mode at the start of the program is set to EM3. However once we start the I2C transaction we add the EM1 power manager requirement and remove it once the transaction completes. I added breakpoints in the I2C0 ISR to check that the microcontroller wakes up to the I2C transaction until it is complete. This can be seen from the spikes in the waveform as can be seen from the image below.
   ![Full_transfer](/Screenshots/assignment_4/Full_transfer.jpg)   

