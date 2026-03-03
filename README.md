# Empptyproject

rebuild of [OrbweaversProsCodeLatest-2](https://github.com/Lorentzsam/OrbweaversProsCodeLatest-2)

## Is This Working
- New updates-controller "unconnected"
- The controler do connected to the brain but the robot won't active to any actions
- still figuring out how to fix that



## To do:
- [x] Make this file fancy :)
- [x] Upload code
    - [x] couputer #1
    - [x] computer #2
    - [x] computer #3
- [x] Fix Driving
    - [x] Motors are good
    - [X] Code can't ruuun
    - [X] Is controller connected and can output joystick status
    - [X] Initialization fail?
    - [X] The screen fuctions
    - [X] More problems
    - [x] rebuild
- [ ] Add the new motors for orb intaking
    - Wait They have not installed it
- [ ] Controler adjust
    - [ ] Button reset
    - [ ] add a swich to the wing(PID also useful)
- [ ] Try moving on its own
    - [x] At least it can move.
    - [ ] PID function
        - [x] Basic PID function
        - [ ] Self-fix
    - [ ] timed run routine
    - [ ] Geometric position get
- [ ] (for debugging) Output some data on the screen
    - [ ] position
    - [x] controller
    - [ ] more!
- [ ] To make auton run useable:
    1. [ ] Get data from the motor
    2. [ ] Use the data to write the code
    3. [ ] Make specific routes for the car

## Debugging
In main.cpp
- added a bool variable called FUNCTION_AS_PREDICTED, and exit code.
- press button A to check if the controller is binded to the partner controller.
## Compiling
- fuck microsoft
- It nuked my compile toolchain
- You will need Pros addon on VS code to compile and upload it
- Make sure the ports are correct and it is a vex V5 brain
    
