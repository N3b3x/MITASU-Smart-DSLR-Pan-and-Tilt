#include "../inc/MotionProcessor.hpp"
#include "math.h"
#include "../inc/PinDef.h"

#define DEBUG   0
#define VERBOSE 0

//=========================================//
//             HELPER FUNCTIONS            //
//=========================================//

/**
 * @brief Returns angle of dy/dx as a value from 0 to 2PI
 * 
 * @param dy 
 * @param dx 
 * @return (double) angle of dy/dx
 */
static double atan3(double dy, double dx) {
    double a = atan2(dy,dx);
    if(a<0) a = (PI*2.0)+a;
    return a;
}

//=========================================//
//               INITIALIZERS              //
//=========================================//

MotionProcessor* MotionProcessor::instance;

MotionProcessor* MotionProcessor::getInstance()
{
    if(instance == NULL){
        instance = new MotionProcessor();
    }
    return instance;
}

MotionProcessor::MotionProcessor(void){
    // Initialize Timer in order to use its functionalities
    Timer1.intialize();

    // Initialize Stepper Motor Drivers and their respective endstops
    // Initialize Stepper Motors disabled off
    motors[0].init(PAN_DIR_PIN, PAN_STEP_PIN, PAN_EN_PIN, EN_MOTOR_OFF, PAN_HALL_PIN);
    motors[1].init(TILT_DIR_PIN, TILT_STEP_PIN, TILT_EN_PIN, EN_MOTOR_OFF, TILT_HALL_PIN, PULLUP_ENDSTOP);
}

//=========================================//
//            MOTION PROCESSORS            //
//=========================================//
/**
 * @brief [BLOCKING] Homes the pan and tilt axis.
 * 
 */
void MotionProcessor::home(){
    enableMotors();

    setPanSpeed(5);
    setTiltSpeed(5);

    #if VERBOSE
    Serial.println("homing...");
    Serial.println("enabled motors");
    #endif

    #if DEBUG
    Serial.print("Pan Step Delay: ");
    Serial.print(_pan_linearStepDelay);
    Serial.println(" us");

    Serial.print("Tilt Step Delay: ");
    Serial.print(_tilt_linearStepDelay);
    Serial.println(" us");
    #endif

    #if VERBOSE
    Serial.println("Homing Pan");
    #endif

    //--------------------------------------------//
    //                HOME PAN
    //--------------------------------------------//
    // Rotate pan clockwise until magnet is first detected
    while (motors[0].endstop()!=HALL_MAG_DETECTED)
    {
        motors[0].hardStep(PAN_DIR_CW);
        pause(_pan_linearStepDelay);
    }

    // Once, magnet is detected, keep rotating untill magnet is no longer detected
    // while counting the steps taken. 
    uint8_t steps_taken_pan = 0;
    while (motors[0].endstop()==HALL_MAG_DETECTED)
    {
        motors[0].hardStep(PAN_DIR_CW);
        steps_taken_pan++;
        pause(_pan_linearStepDelay);
    }

    // Once magnet is no longer detected, we need to move back half of the steps we've taken
    // to land on the center of the magnet
    for(int i = 0; i<(steps_taken_pan/2); i++){
        motors[0].hardStep(PAN_DIR_CCW);
        pause(_pan_linearStepDelay);
    }
    
    if VERBOSE
    Serial.println("Finished homing pan")
    Serial.println("Homing Tilt...");
    #endif
    //--------------------------------------------//
    //                HOME TILT
    //--------------------------------------------//
    // Rotate pan clockwise until magnet is first detected
    while (motors[1].endstop()!=HALL_MAG_DETECTED)
    {
        motors[1].hardStep(TILT_DIR_CW);
        pause(_tilt_linearStepDelay);
    }

    // Keep rotating untill magnet is no longer detected while counting the steps taken 
    uint8_t steps_taken_tilt = 0;
    while (motors[1].endstop()==HALL_MAG_DETECTED)
    {
        motors[1].hardStep(TILT_DIR_CW);
        steps_taken_tilt++;
        pause(_tilt_linearStepDelay);
    }

    // Once magnet is not longer detected, we need to move back half of the steps we've taken
    // to land on the center of the magnet
    for(int i = 0; i<(steps_taken_tilt/2); i++){
        motors[1].hardStep(TILT_DIR_CCW);
        pause(_tilt_linearStepDelay);
    }

    setPosition(doubleZeroVect);

    if VERBOSE
    Serial.println("Finished homing tilt")
    Serial.println("Homing Finished!!");
    #endif
}

/**
 * @brief [BLOCKING] Do a linear movement from starting to end points. 
 * All stepper motors finish movement at the same time. However,
 * this function returns only when it reaches the destination.
 * 
 * @param coords holds the goal values for each axis
 */
void MotionProcessor::dumbLine(DoubleVector coords){

}

/**
 * @brief [NON-BLOCKING] Do a linear movement from starting to end points. 
 * All stepper motors finish movement at the same time.
 * Service stepper motors through interupt service routine.
 * 
 * @param coords holds the goal values for each axis
 * @param d holds the direction we want to 
 */
void MotionProcessor::line(DoubleVector coords){
    long stepDelay;
    int i;

    // Calculate the number of steps that need to be taken to achieve 80 degrees of rotation
    // [# degrees]/[# degrees/step] = [# step]
    _delta[0] = coords.p / PAN_STEPRATE;     
    _delta[1] = coords.t / TILT_STEPRATE;

    // If we're in relative mode, the amount of steps we need to take to achieve 80 degrees of rotation is what's calculated above.
    // However, if we're in absolute mode, substract the amount of steps we have already taken from home 
    if(_mode == ABS){
        _delta[0] -= _currentPositionSteps.p;
        _delta[1] -= _currentPositionSteps.t;
    }

    #if VERBOSE || DEBUG
    Serial.println("line command:");
    Serial.print("delta p: ");
    Serial.print(delta[0]);
    Serial.print("\t");
    Serial.print("delta t: ");
    Serial.println(delta[1]);
    #endif

    // To run the bresenham algorithm, we need to find the axis with the biggest delta which is also the fastest one.
    // The axis with the biggest delta will be the one continuouly stepped while we determine if the 
    // other one gets stepped or not.
    // Find axis with the biggest delta
    // We'll first inialize the fastest motor index to be 0. We then step through each delta and if the index getting checked
    // has a higher delta, update the fastest motor's index to that one.
    _fastest = 0;
    for (i = 0; i < NUM_OF_MOTORS; i++){
        if( abs(_delta[_fastest]) <= abs(_delta[i]) ){
            _fastest = i;
        }
    }

    #if DEBUG
    Serial.print("Fastest axis: ");
    Serial.print(_fastest);
    #endif

    // Let's set the direction for the pan motor
    if(_delta[0] < 0){
        motors[0].setDir(PAN_DIR_CW); 
    }
    else{
        motors[0].setDir(PAN_DIR_CCW);
    }

    // Let's set the direction for the tilt motor
    if(_delta[1] < 0){
        motors[1].setDir(TILT_DIR_CW); 
    }
    else{
        motors[1].setDir(TILT_DIR_CCW);
    }
    

}

/**
 * @brief Do a iteration of the bresenham algorithm
 * @note Is a public static to be able to be attached to an interrupt
 * 
 */
static void MotionProcessor::bresenham(void){

}


//=========================================//
//           SETTERS AND GETTERS           //
//=========================================//

/**
 * @brief Sets the absolute position of the pan and tilt axis
 * 
 * @param p pan axis position
 * @param t tilt axis position
 */
void MotionProcessor::setPosition(double p, double t){
    DoubleVector pos = {p,t};
    setPosition(pos);
}

/**
 * @brief Sets the absolute position of the pan and tilt axis
 * 
 * @param pos position vector of the pan and tilt
 */
void MotionProcessor::setPosition(DoubleVector pos){

    _currentPosition = pos;
    _currentPositionSteps.p = (long)(pos.p / PAN_STEPRATE);
    _currentPositionSteps.t = (long)(pos.t / TILT_STEPRATE);
}

/**
 * @brief Gets the absolute position of the pan and tilt axis
 * 
 * @return Position vectors as a DoubleVector
 */
DoubleVector MotionProcessor::getPosition( void ){
    return _currentPosition;
}
/**
 * @brief Get how many steps the stepper motor have taken thus far from home
 * 
 * @return Number of steps as a LongVector 
 */
LongVector MotionProcessor::getPositionSteps(void){
    return _currentPositionSteps;
}

/**
 * @brief Set the panning Speed
 * 
 * @param speed [Degrees/Sec]
 */
void MotionProcessor::setPanSpeed(double speed){
    if (speed < PAN_MIN_SPEED)
    {
        #if DEBUG
        Serial.print("too little Speed. Setting to ");
        Serial.print(PAN_MIN_SPEED);
        Serial.println(" degree/s");
        #endif
        _pan_speed = PAN_MIN_SPEED;
    }

    else if (speed > PAN_MAX_SPEED)
    {
        #if DEBUG
        Serial.print("too much Speed. Setting to ");
        Serial.print(PAN_MAN_SPEED);
        Serial.println(" degree/s");
        #endif
        _pan_speed = PAN_MAX_SPEED;
    }
    else{
        _pan_speed = speed;
    }

    _pan_feedrate =  _pan_speed/PAN_STEPRATE;                   //steps per second
    _pan_linearStepDelay = (long)(1000000.0 / _pan_feedrate);   //microseconds

    #if DEBUG
    Serial.print("feedrate: ");
    Serial.print(_pan_feedrate);
    Serial.println(" steps/s");
    Serial.print("linear step delay: ");
    Serial.print(_pan_linearStepDelay);
    Serial.println(" us");
    #endif
}

/**
 * @brief Set the tilting speed
 * 
 * @param speed [Degrees/Sec]
 */
void MotionProcessor::setTiltSpeed(double speed){
       if (speed <= TILT_MIN_SPEED)
    {
        #if DEBUG
        Serial.print("too little Speed. Setting to ");
        Serial.print(TILT_MIN_SPEED);
        Serial.println(" degree/s");
        #endif
        _tilt_speed = TILT_MIN_SPEED;
    }

    else if (speed >= TILT_MAX_SPEED)
    {
        #if DEBUG
        Serial.print("too much Speed. Setting to ");
        Serial.print(TILT_MAX_SPEED);
        Serial.println(" degree/s");
        #endif
        _tilt_speed = TILT_MAX_SPEED;
    }
    else{
        _tilt_speed = speed;
    }

    _tilt_feedrate =  _tilt_speed/TILT_STEPRATE;                 //steps per second
    _tilt_linearStepDelay = (long)(1000000.0 / _tilt_feedrate); //microseconds

    #if DEBUG
    Serial.print("feedrate: ");
    Serial.print(_tilt_feedrate);
    Serial.println(" steps/s");
    Serial.print("linear step delay: ");
    Serial.print(_tilt_linearStepDelay);
    Serial.println(" us");
    #endif
}

/**
 * @brief Get the panning speed
 * 
 */
void MotionProcessor::getPanSpeed(){
    return _pan_speed;
}

/**
 * @brief Get the tilting speed
 * 
 */
void MotionProcessor::getTiltSpeed(){
    return _tilt_speed;
}

/**
 * @brief Get the pan stepper motor feedrate
 * 
 */
void MotionProcessor::getPanFeedrate(){
    return _pan_feedrate;
}

/**
 * @brief Get the tilt stepper motor feedrate
 * 
 */
void MotionProcessor::getTiltFeedrate(){
    return _tilt_feedrate;
}

/**
 * @brief Set the moving mode between Absolute and Relative
 * 
 * @param mode ABS for absolute, REL for relative
 */
void MotionProcessor::setMode( MoveMode mode){
    _mode = mode;
}

/**
 * @brief Get the movement mode
 * 
 * @return MoveMode 
 */
MoveMode MotionProcessor::getMode(){
    return _mode;
}

/**
 * @brief Enable Motors
 * 
 */
void MotionProcessor::enableMotors(void){
    for(int i = 0; i<NUM_OF_MOTORS; i++){
        motors[i].enable();
    }
}

/**
 * @brief Disable Motors
 * 
 */
void MotionProcessor::disableMotors(void){
    for(int i = 0; i<NUM_OF_MOTORS; i++){
        motors[i].disable();
    }
}

/**
 * @brief [BLOCKING] pauses code for a specific amount of time
 * 
 * @param us microseconds to be paused
 */
void MotionProcessor::pause(long us){
    // Break it into delay and delay microseconds because delayMicroseconds doesn't
    // work accurately for values over 16383. Check it out at link below.
    // https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/
    delay(us/1000);
    delayMicroseconds(us%1000);
}

/**
 * @brief Probes if system is ready to receive new commands
 * 
 * @return uint8_t 
 */
uint8_t MotionProcessor::ready(void){
    return _ready;
}

/**
 * @brief Register try and executes
 * 
 * @param tryAndExecCallback 
 */
void MotionProcessor::registerTryAndExecCallback(void (*tryAndExecCallback)(void)){
    _tryAndExecCallback = tryAndExecCallback;
}