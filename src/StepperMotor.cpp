#include "../inc/StepperMotor.hpp"

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Creates a stepper motor class
 * ----------------------------------------------------------------------------------------------------------------------------------
*/
StepperMotor::StepperMotor(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin, uint8_t endstop_pin, pinSetup_t endstop_pin_setup = NONE)
{
    _dir_pin = dir_pin;
    _step_pin = step_pin;
    _en_pin = en_pin;
    _endstop_pin = endstop_pin;

    pinMode(_dir_pin, OUTPUT);
    pinMode(_step_pin, OUTPUT);
    pinMode(_en_pin, OUTPUT);

    if (endstop_pin_setup == NONE)
        pinMode(_endstop_pin, INPUT);
    else if (endstop_pin_setup == PULLUP_ENDSTOP)
        pinMode(_endstop_pin, INPUT_PULLUP);
}

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Enables stepper motor
 * ----------------------------------------------------------------------------------------------------------------------------------
*/
void StepperMotor::enable(void)
{
    digitalWrite(_en_pin, EN_MOTOR_ON);
}

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Disables stepper motor
 * ----------------------------------------------------------------------------------------------------------------------------------
*/
void StepperMotor::disable(void)
{
    digitalWrite(_en_pin, EN_MOTOR_OFF);
}

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Performs a step for the stepper motor considering the endstop. 
 *         If endstop is being pressed, return. Else step.
 * 
 *  @param[in] dir Value to set direction pin
 * 
 *  @return 0 if step not taken due to reaching endstop, and
 *          1 if step was taken
 * ----------------------------------------------------------------------------------------------------------------------------------
*/
uint8_t StepperMotor::step(uint8_t dir)
{
    if (endstop())
        return 0;

    hardStep(dir);
    return 1;
}

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Performs a step for the stepper motor without considering endstop
 * 
 *  @param[in] dir Value to set direction pin
 * ----------------------------------------------------------------------------------------------------------------------------------
*/
void StepperMotor::hardStep(uint8_t dir)
{
    digitalWrite(_dir_pin, dir);

    digitalWrite(_step_pin, HIGH);
    digitalWrite(_step_pin, LOW);
}

/**
 * ----------------------------------------------------------------------------------------------------------------------------------
 *  @brief Reads the endstop value for the associated stepper motor
 * 
 *  @return 1 if pressed/activated
 *          0 if not pressed/activated
 * ----------------------------------------------------------------------------------------------------------------------------------
*/
uint8_t StepperMotor::endstop(void)
{
#ifdef DEBUG
    Serial.print("Endstop: ");
    Serial.println(!digitalRead(_end_pin));
#endif

    return !digitalRead(_endstop_pin); // Inverted because 0 is active since endstops are setup as input pullup
}
