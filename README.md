# Arduino Elego Robot

This repository contains code for arduino based rover used in an intoductory course (CSCI 1063U – Computer Programming Project) at the Ontario Tech University. 

## Hardware Assumptions

The code is specifically written for `ELEGOO Smart Robot Car Kit V4.0`

The particular unit used by our group had the following constraints 

- The left wheel is around 1.17% weaker than the right (requiring the analog output to be 3 units higher)

- That the gyroscope is sometimes unreliable needing to turn based on timing which are manually calibrated.
- The line detection left sensor value is 170 units weaker 

## Code base Structure

- `6_7/6_7.ino` -> Experiment code that draws the number 67. Can safely ignore
- `Base_code/` 
  - `Base_code.ino` -> Contains main logic for solving maze.
  - `pin_def.h` -> Pin definitions for sensors and motors.
  - `utils.cpp` & `utils.h` -> Utility functions used by `Base_code.ino`. Contains all the sensor functions
- Additional Libraries -> `FastLED`: used for LED control (already included for reference).

## Base_code Folder

The `Base_code/Base_code.ino` is intended as a starting point for new funtinality. Treat it as the main file. `pin_def.h` defines the pin mappings. Any additional sensors need to be included there. `utils.cpp` & `utils.h` are intended to include functions and APIs for new sensor that the `Base_code.ino` can call. By de-centralizing code into multiple files, maintenance and updates become easier.

### Current logic
Currently the code is intended for the rover to traverse and solve a distinct bi-color maze. The rover uses the IR sensor to measure the light intensity being reflected from the ground material. Darker ground/line gives lower values and lighter gives higher values. 

The robot will go forward until it encounters a  bright color surface below it. If the all the all three sensor touch the bright color object, then it will signal an 90 degree turn. If the rover drifts into the maze lining in which case eiter left or right sensor will get trigered, the rover will correct it's corse accordingly. 

For the sake of solving the maze the rover randomly decides on a 50% probability wheater to make a right or left 90 degree turn.

`Inline comments are used heavily to accomodate beginers and explain specific funtionality further`

`Feel free to adapt or expand the base code according to your projects need.`

## 6_7 Folder

The 6_7 Folder is purely for the memes and only has one purpose when run to draw out the numbers 67 in sequence (with regards to the hardware assumptions). This has almost no realistic use beyond this.

## Clone to Run

### 1. Cloning the Repo:

     In the terminal run:
`git clone https://github.com/mashfaq9090/Arduino_Elego_Robot`

### 2. Installing Arduino IDE:

  Install based on you device specifications and operating system at:

    [Arduino IDE](https://www.arduino.cc/en/software/)

### 3. IDE setup

- From Board Manager -> Install Arduino AVR Broads if not already installed (1.8.7 or later)
- From Library Manager -> install FastLED (3.10.3 or later)
- Optional: File -> Preferences -> Check both for; show verbose output during compile and upload

### 4. Board setup

    On the top left of the IDE click `Select Board`
    if Unkown on COMX is visible click on it otherwise click on __Select other board and port...__ then set board to Arduino UNO.

### 5. Pre-run Test

     Attempt to compile/Verify with the check mark in the top left corner (or with Ctrl + R). If this passes plug in robot ensure that the board and port are both recognized then  upload to test robot. (If port issues arise refer to (https://www.youtube.com/watch?v=D271p2E2_o4)).

### 6. Troubleshooting and adjusting

    With debug functions inside base_code.ino you can debug and find issues unique to your robot.

## Known issues for our specific unit

- The right motor is slightly stronger than the left  and the left color value is a bit higher this is reflected in base_code.ino file.
- Not all functionality of the ELEGOO Smart Robot Car Kit V4.0 is used in our code. 
