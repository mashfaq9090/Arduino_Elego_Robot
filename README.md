# Arduino Elego Robot

This repository contains code for an Elego line-following robot project using an Arduino. The project is organized into several sketches and a `Base_code` folder which holds shared definitions and utilities.

## Hardware Assumptions

- Used ardunio kit is ELEGOO Smart Robot Car Kit V4.0 (or similar model with An Ardunio UNO Board)  
- The left wheel is around 1.17% weaker than the right (requiring the analog output to be 3 higher)
- That the gyroscope is sometimes unreliable needing to turn based on timing.
- The line detection left sensor value is 170 weaker on average and that the colour you would like to avoid is greater than 500 in sensor value.

## Structure

- `6_7/6_7.ino` - Additional experiment code .
- `Base_code/` - Contains common code used across sketches:
  - `Base_code.ino` - Example base sketch that includes shared setup/loop structure.
  - `pin_def.h` - Pin definitions for sensors and motors.
  - `utils.cpp` & `utils.h` - Utility functions used by multiple sketches.
- `libraries/FastLED/` - FastLED library for LED control (already included for reference).

## Base_code Folder

The `Base_code` folder is intended as a starting point for new sketches. It defines the pin mappings and utility routines so users can copy or include these files in their own projects. By centralizing common code here, maintenance and updates become easier.

To start with the robot will go forward until it encounters a wall by sweeping its servo, checking its surroundings / or a bright color surface below it then with turn to avoid it  

Feel free to adapt or expand the base code as your project evolves.

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

## Known issues

- The right motor is slightly stronger than the left  and the left color value is significantly this is reflected in base_code.ino file.
- With the ELEGOO Smart Robot Car Kit V4.0 the camera and all its functionallity is not used inside any of the code
