This is a C project for *STM32F103C8T6*


Core features:
1. Show time
2. Show weather (pressure, temperature, humidity)
3. Timer with alarm (sound and light)


Pins:
1. Screen (GME12864):
   - SCL - B6
   - SDA - B7

2. Weather (BME280):
   - SCL - B10
   - SDA - B11

3. Encoder (KY-040):
   - CLK - A15
   - DT - B3
   - SM - not used

4. Alarm (LED + Buzzer):
   - A1 

6. Buttons:
   - First - A3
   - Second - B1

6. Battery for timer
   - G - 3v battery -
   - 3VB - 3v battery +


Manual:
1. Time screen
   - button 1 - will send to menu screen
   - button 2 - will send to weather screen
   - encoder - will send to timer screen 
3. Weather screen
   - button 2 - will send to timer screen
5. Timer screen
   - button 2 - will send to time screen
   - encoder - will set desired delay in minutes
   - button 1 - if the selected delay is not 0, will start the countdown. Click again to stop countdown and clear.
   - button 1 or button 2 when the alarm is on - will stop alarm 
7. Menu screen
   - encoder - change selection. It is useless for now, because there is only one active option - set time.
   - button 1 - go back
   - button 2 - go to selected option
9. Set time screen
    - encoder - set number of seconds, minutes, hours
    - button 2 - change selection (seconds, minutes, hours)
    - button 2 when everything is set up - go back
