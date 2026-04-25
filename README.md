# Arduino Elego Robot

This repository contains code for arduino based rover used in an intoductory course (CSCI 1063U – Computer Programming Project) at the Ontario Tech University. 

## Hardware Assumptions

The code is specifically written for `ELEGOO Smart Robot Car Kit V4.0`

The particular unit used by our group had the following constraints 

- The left wheel is around 1.17% weaker than the right (requiring the analog output to be 3 units higher)

- That the gyroscope is sometimes unreliable needing to turn based on timing which are manually calibrated.
- The line detection left sensor value is 170 units weaker 

## Code base Structure
- `Base_code/` 
  - `Base_code.ino` -> Contains main logic for solving maze.
  - `pin_def.h` -> Pin definitions for sensors and motors.
  - `utils.cpp` & `utils.h` -> Utility functions used by `Base_code.ino`. Contains all the sensor functions
  - multiple .h and .cpp for custom logic. The file name are self explanatory !!!
- Additional Libraries -> `FastLED`: used for LED control (already included for reference).

## Base_code Folder

The `Base_code/Base_code.ino` is intended as a starting point for new funtinality. Treat it as the main file. `pin_def.h` defines the pin mappings. Any additional sensors need to be included there. `utils.cpp` & `utils.h` are intended to include functions and APIs for new sensor that the `Base_code.ino` can call. By de-centralizing code into multiple files, maintenance and updates become easier.

## Final Logic

This repo is our official code for solving a `sonar` based map autonomously. For our course finals we were required to solve two maze that were linked by a narrow white trail. 

#### Sensor Used
We used all of our rover sensor to it's maximum potential. 

- Sonar  --------------------> used for decision making
- Gyroscope -----------------> used for precise turning
- Line sensor ---------------> used for traversing the white trail. 

## Maze solving Logic

**First Maze**: For the first maze we decided to do small incremental steps and at each step fetch left and right distance data to make decisions weather a local branch was nearby. We call this `The turtle walk 🐢 method!!`. Based on visual inspection, we have realized this to be the most efficient way to travese a branch that directly leads us to the `narrow white trail`. 

**Sencond Maze**: For the second maze we had a left biased logic, which has 2 mode. If either the left or the right distance was way to close we call it a `forced choice`, where the rover goes whichever is open. But if it see's both left and right and viable option we always choses left. Again we decided this logic to be the most optimal one based on visual inspection


The above two logic are specific to how our instructor set up the maze :(
However they are not true MAZE SOLVER. 

**Our vission for a true Maze Solver** (if we had infinite resource)
Initially we were young and too ambitious. We had a vission to create a true SLAM maze solver. Our philosopy was if we record each and every decission in a struct in an array....there will be some logic out there that can solve the maze purely from seeing that struct. 

Our struc would look like the following

```C++ 
//=============================================================
// MAZE SOLVER — Data Structure 
//=============================================================

// ---- Distance thresholds ----
#define DEFINITELY_OPEN  20    // cm — treat as open path
#define DEFINITELY_WALL  8     // cm — treat as wall

// ---- Direction constants ----
#define DIR_NONE   0x00
#define DIR_FWD    0x01
#define DIR_RIGHT  0x02
#define DIR_LEFT   0x04
#define DIR_BACK   0x08
/*
bit position:  7  6  5  4  3  2  1  0
               0  0  0  0  B  L  R  F
                           │  │  │  │
                           │  │  │  └── bit0 = FORWARD  (0x01)
                           │  │  └───── bit1 = RIGHT    (0x02)
                           │  └──────── bit2 = LEFT     (0x04)
                           └─────────── bit3 = BACK     (0x08)

*/

// ---- Alt direction constants ----
#define ALT_NONE   'N'
#define ALT_LEFT   'L'
#define ALT_RIGHT  'R'

// ---- Junction type constants ----
#define TYPE_FORCED  'F'
#define TYPE_CHOICE  'C'

#define IS_OPEN(dist) (dist > 8)
#define IS_WALL(dist) (dist <= 8)

// ---- Junction struct ----
struct Junction {
    uint8_t  tried_dirs;  // bitmask of attempted directions
    uint8_t  last_turn;   // turn made TO ARRIVE at this junction
    uint16_t travel_ms;   // ms driven to reach here
    char  type;        // TYPE_FORCED or TYPE_CHOICE
    char  alt_dir;     // ALT_NONE, ALT_LEFT, ALT_RIGHT
    uint8_t  alt_dist;    // distance of alt direction in cm
};                        // 7 bytes per junction

// ---- Stack ----
const uint8_t MAX_JUNCTIONS = 30;
Junction jstack[MAX_JUNCTIONS];  // 60 × 7 = 420 bytes
uint8_t  stack_top = 0;          // current depth
uint8_t last_maze_turn = DIR_NONE;

// ---- Timing ----
unsigned long drive_start_ms = 0;

// ---- Loop detection ----
uint8_t turnLog       = 0;
bool    loop_break    = false;

//=============================================================
// MAZE SOLVER — Data Structure ------------------------------>>>>>>>>>> END
//=============================================================

```

But unfortunately we were running dangerously low on dynamic memory and had to scrath that idea. It was simply too much more our little robot. Plus we hadn't completly figured out what we would do with the struct. We had couple of ideas of `backtraking` we we see a brach make a parralel array and do some kind of checking ect ect...


`Inline comments are used heavily to accomodate beginers and explain specific funtionality further`

`Feel free to adapt or expand the base code according to your projects need.`
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
