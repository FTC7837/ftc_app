package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Thundercolts7837 on 9/24/2015.
 */

public class HangBotManual extends OpMode {
    final static double ARM_MIN_RANGE  = 0.1;
    final static double ARM_MAX_RANGE  = 0.99;
    final static double CLAW_MIN_RANGE  = 0.01;
    final static double CLAW_MAX_RANGE  = 0.7;

    // position of the arm servo.
    double armPosition;

    // amount to change the arm servo position.
    double armDelta = 0.005;

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.005;
    boolean SlowCurrent = false;
    boolean SlowPrevious = false;

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorArm;
    DcMotor motorLift;
    //Servo claw;
    //Servo arm;

    /**
     * Constructor
     */
    public HangBotManual() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_right" and "motor_left"
		 *   "motor_right" is on the right side of the bot.
		 *   "motor_left" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorArm = hardwareMap.dcMotor.get("motor_arm");
        motorLift = hardwareMap.dcMotor.get("motor_lift");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        //arm = hardwareMap.servo.get("servo_1");
        //claw = hardwareMap.servo.get("servo_6");

        // assign the starting position of the wrist and claw
        armPosition = 0.2;
        clawPosition = 0.2;
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {



		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        // Tank Drive Code
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        SlowCurrent=gamepad1.right_bumper;
        if ((SlowCurrent) && (!SlowPrevious) )//rising edge
        {
            SlowCurrent = !SlowCurrent;//toggle value
        }
        SlowPrevious = SlowCurrent;

        if (gamepad1.left_bumper) {
            right = left;//Lock Differentials
        }

        if (gamepad1.dpad_up)
        {
            motorLift.setPower(0.8);//Extend Arm
        }
        else if(gamepad1.dpad_down)
        {
           motorLift.setPower(-0.8); //Retract Arm
        }
        else
        {
            motorLift.setPower(0);//nothing was pressed
        }


        if (gamepad1.left_trigger > 0.04) {//move arm down
            motorArm.setPower(-gamepad1.left_trigger/2);
        }
       else if (gamepad1.right_trigger > 0.04) {//move arm up
            motorArm.setPower(gamepad1.right_trigger/2);
        }
        else
        {
            motorArm.setPower(0);//don't run arm
        }


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);


        if (SlowCurrent) {
            right/=4;
            left/=4;
        }


        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);

        // update the position of the arm.
        if (gamepad1.a) {
            // if the A button is pushed on gamepad1, increment the position of
            // the arm servo.
            armPosition += armDelta;
        }

        if (gamepad1.y) {
            // if the Y button is pushed on gamepad1, decrease the position of
            // the arm servo.
            armPosition -= armDelta;
        }

        // update the position of the claw


        if (gamepad1.b) {
            clawPosition -= clawDelta;
        }

        // clip the position values so that they never exceed their allowed range.
        armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

        // write position values to the wrist and claw servo
        //arm.setPosition(armPosition);
        //claw.setPosition(clawPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}



/*
*
*bool shift = false;

if(joy1Btn(1) {
   if(!shift)
   {
       //action
   }
   shift = true;
}
else {shift = false;}
*
*
* int state=0;
int lastButton=0;
int currButton;

while(1 == 1) {
	getJoystickSettings(joystick);

        currButton=joy1Btn(1);
        if( (currButton==1) && (lastButton==0) ) {

            if(state>0) {
	        state=0;
	    }
            else {
	        state=20;
	    }
        }
        lastButton=currButton;
	motor[mtr_S1_C1_1] = state;
	wait1Msec(5);
}
*
*
*
* */

