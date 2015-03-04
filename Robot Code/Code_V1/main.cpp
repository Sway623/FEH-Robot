#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>

//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_1);
FEHMotor right_motor(FEHMotor::Motor0);
FEHMotor left_motor(FEHMotor::Motor1);
FEHServo servo(FEHServo::Servo7);
FEHServo servoSalt(FEHServo::Servo4);
AnalogInputPin CdS(FEHIO::P0_0);

//declares RPS location constants for key locations
const float CRANK_X;
const float CRANK_Y;
const float SALT_X;
const float SALT_Y;
const float BUTTONS_X;
const float BUTTONS_Y;
const float GARAGE_X;
const float GARAGE_Y;
const float SWITCH_X;
const float SWITCH_Y;

const int percent = 60; //sets the motor percent for the rest of the code
const int toSlow = 25; //this int will be the fix required for the robot to travel among the course
const float cts_per_in= 3.704; //counts per inch
const float cts_per_deg; //counts per degree

//declares prototypes for functions
void goToCrank();
void goToButtons();
void goToSalt();
void goToGarage();
void goToSwitch();

void turnCrank();
void pushButtons();
void getSalt();
void depositSalt();
void toggleSwitch();

void move(int percent, int counts);
void turn_left(int percent, int counts);
void turn_right(int percent, int counts);

/*
 *This method allows the robot to move for a certain number of counts
 * @param counts
 *              the number of counts for which the robot should move forward
 * @param percent
 *              the motor power percentage at which the robot will travel.
 * @convention
 *              if @param percent is negative, robot will move backwards
 *              else robot will move forwards
 */
void move(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
} //move

/*
 *This method allows the robot to turn right for a certain number of counts
 * @param counts
 *              the number of counts for which the robot should be turning
 * @param percent
 *              the percent at which the motor will be powered during turning
 */
void turn_right(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    //hint: set right motor forward, left motor backward

    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);


    //While the average of the left and right encoder are less than counts,
    //keep running motors

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
} //turn_right

/*
 *This method allows the robot to turn left for a certain number of counts
 * @param counts
 *              the number of counts for which the robot should be turning
 * @param percent
 *              the percent at which the motor will be powered during turning
 */
void turn_left(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent

    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
} //turn_left

/*
 *This method will allow the robot to push the buttons on the side of the BOO in order
 */
void pushButtons(){
    int counts = cts_per_in * 1;
    if (RPS.RedButtonOrder() == 1){
        servo.SetDegree(0); //prepare to hit red button
        move(percent-toSlow, counts); //drive forward and push button
        move(-(percent-toSlow), counts); //back up
        if (RPS.WhiteButtonOrder() == 2){
            servo.SetDegree(60); //prepare to hit white button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
            servo.SetDegree(120); //prepare to hit blue button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
        } else{ //if blue is second
            servo.SetDegree(120); //prepare to hit blue button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
            servo.SetDegree(60); //prepare to hit white button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
        } //else
    } else if (RPS.WhiteButtonOrder() == 1){
        servo.SetDegree(60); //prepare to hit white button
        move(percent-toSlow, counts); //drive forward and push button
        move(-(percent-toSlow), counts); //back up
        if (RPS.RedButtonOrder() == 2){
            servo.SetDegree(0); //prepare to hit red button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
            servo.SetDegree(120); //prepare to hit blue button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
        } else{ //if blue is second
            servo.SetDegree(120); //prepare to hit blue button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
            servo.SetDegree(0); //prepare to hit red button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
        } //else
    } else if (RPS.BlueButtonOrder() == 1){
        servo.SetDegree(120); //prepare to hit blue button
        move(percent-toSlow, counts); //drive forward and push button
        move(-(percent-toSlow), counts); //back up
        if (RPS.RedButtonOrder() == 2){
            servo.SetDegree(0);
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
            servo.SetDegree(60); //prepare to hit white button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
        } else{ //if white is second
            servo.SetDegree(60); //prepare to hit white button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
            servo.SetDegree(0); //prepare to hit red button
            move(percent-toSlow, counts); //drive forward and push button
            move(-(percent-toSlow), counts); //back up
        } //else
    }
} //pushButtons

/*
 * This method will pick up the salt bag.
 * The salt bag is to be dragged by the robot for the remainder of the course,
 * until it is deposited in the garage.
 */
void getSalt(){
    servoSalt.SetDegree(90);
} //getSalt

/*
 * This method will deposit the salt into the garage.
 */
void depositSalt(){
    servoSalt.SetDegree(0);
    move(-(percent-35), counts); //back up and push salt into garage
} //depositSalt

/*
 * This method will toggle the oil switch.
 * @convention
 *          if RPS.OilDirec() is 0, then switch must be pushed to the left
 *          else the switch must be pushed to the right.
 */
void toggleSwitch(){
    if (RPS.OilDirec()==0){
        turn_right(percent-toSlow, cts_per_deg*45);
        servoSalt.SetDegree(45);
        turn_left(percent-toSlow, cts_per_deg*50);
    } else{
        turn_left(percent-toSlow, cts_per_deg*45);
        servoSalt.SetDegree(45);
        turn_right(percent-toSlow, cts_per_deg*50);
    }
} //toggleSwitch

/*
 * This method will take the robot from the start light, to the salt bag.
 */
void goToSalt(){

} //goToSalt

/*
 * This method will take the robot from the salt bag to the crank.
 */
void goToCrank(){

} //goToCrank

/*
 * This method will take the robot from the crank to the buttons
 */
void goToButtons(){

} //goToButtons

/*
 * This method will take the robot from the buttons, to the garage.
 * After this method is called, the robot should be in good position
 * to deposit the salt into the garage.
 */
void goToGarage(){

} //goToGarage

/*
 * Finally, this method will take the robot from the garage, to the oil switch.
 * After this method is called, the robot should go down the avalanche ramp and
 * be placed in a good position to toggle the switch.
 */
void goToSwitch(){

} //goToSwitch

/*
 * This method is written in order to efficiently complete performance test 3
 */
void performanceTest3(){
    move(percent, cts_per_in*13); //move 13 inches forward
    turn_left(percent- toSlow, cts_per_deg*90); //turn left 90 degrees
    move(percent, cts_per_in*11); //move 11 inches forward
    turn_left(percent - toSlow, cts_per_deg*90); //turn left 90 degrees
    move(percent, cts_per_in*34); //move 34 inches forward
    turn_left(percent-toSlow, cts_per_deg*45); //turn left 45 degrees
    move(percent, cts_per_in*22); //move 22 inches forward
    pushButtons();
} //performanceTest3

/*
 * The main method
 */
int main(void)
{
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    const arrayLength = 5; //sets length of task array

    /*
     * Initialize task array.
     * Elements of which are to be used in switch case in main.
     */
    int taskArray[arrayLength] = {0, 1, 2, 3, 4};

    //initialize positions of servo motors
    servoSalt.SetDegree(0);
    servo.SetDegree(0);

    while(CdS.Value()>1);

    performanceTest3();

//    for (int i=0; i<arrayLength; i++){
//        switch (taskArray[i]){
//        case 0:
//            goToSalt();
//            getSalt();
//            break;
//        case 1:
//            goToCrank();
//            turnCrank();
//            break;
//        case 2:
//            goToButtons();
//            turnButtons();
//            break;
//        case 3:
//            goToGarage();
//            depositSalt();
//        case 4:
//            goToSwitch();
//            toggleSwitch();
//        } //switch
//    } //for

    return 0;
} //main
