package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Thundercolts7837 on 9/24/2015.
 */

public class HangBotWEncoder extends OpMode {
    final static double LEFT_PADDLE_RETRACT  = 0.75;
    final static double LEFT_PADDLE_READY = 0.5;
    final static double LEFT_PADDLE_PRESS = 0.35;
    final static double RIGHT_PADDLE_RETRACT  = 0.0;
    final static double RIGHT_PADDLE_READY = 0.3;
    final static double RIGHT_PADDLE_PRESS = 0.45;
    final static int LIFT_MAX = 11450;
    final static int ARM_MAX = 9300;//may change with bungee
    // position of the paddle servo.
    double paddleLeftPosition;
    double paddleRightPosition;

    int xVal, yVal, zVal = 0;
    int heading = 0;
    float beaconhsvValues[] = {0F,0F,0F};
    float linehsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    final float bvalues[] = beaconhsvValues;
    final float lvalues[] = linehsvValues;
    boolean SlowCurrent = false;
    boolean SlowPrevious = false;
    boolean SlowMode = false;
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorArm;
    DcMotor motorLift;
    Servo paddleRight;
    Servo paddleLeft;
    GyroSensor sensorGyro;
    ColorSensor sensorBeacon;
    ColorSensor sensorLine;
    TouchSensor limitLowerLift;
    TouchSensor limitLowerArm;
    boolean limitLowerLiftPrevious=false;
    boolean limitLowerArmPrevious=false;

    /**
     * Constructor
     */
    public HangBotWEncoder() {

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
		 * We also assume that there are two servos "servo_5" and "servo_6"
		 *    "servo_5" controls the right of the manipulator.
		 *    "servo_6" controls the left of the manipulator.
		 */
        motorRight = hardwareMap.dcMotor.get("motor_right");//controller 0
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorArm = hardwareMap.dcMotor.get("motor_arm");
        motorLift = hardwareMap.dcMotor.get("motor_lift");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        paddleRight = hardwareMap.servo.get("servo_right");//Servo 5
        paddleLeft = hardwareMap.servo.get("servo_left");//Servo 6

        // assign the starting position of paddles
        paddleRightPosition = 0.2;
        paddleLeftPosition = 0.2;
        // get a reference to our GyroSensor object.
        sensorGyro = hardwareMap.gyroSensor.get("sensor_gyro");
        // calibrate the gyro.
        sensorGyro.calibrate();




        // get a reference to our ColorSensor object.



        sensorBeacon = hardwareMap.colorSensor.get("sensor_beacon");//port 1 0x42
        sensorBeacon.setI2cAddress(0x42);
        sensorLine = hardwareMap.colorSensor.get("sensor_line");//port 0 0x40
        //sensorLine.setI2cAddress(0x42);
        // turn the LED on in the beginning, just so user will know that the sensor is active.
        sensorBeacon.enableLed(false);
        sensorLine.enableLed(true);

        limitLowerArm = hardwareMap.touchSensor.get("limit_lower_arm");
        limitLowerLift = hardwareMap.touchSensor.get("limit_lower_lift");

        hardwareMap.logDevices();
        // wait for the start button to be pressed.
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {




        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        // Tank Drive Code
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

        float armPower = (float) (gamepad2.left_stick_y*0.6);
        float liftPower = (float) -(gamepad2.right_stick_y*0.8);

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        SlowCurrent=gamepad1.right_bumper;
        if ((SlowCurrent) && (!SlowPrevious) )//rising edge
        {
            if (!SlowMode) SlowMode = true;
            else SlowMode = false;
        }
        SlowPrevious = SlowCurrent;

        if (gamepad1.a) {
            right = left;//Lock Differentials
        }


        //Limit and encoder reset code for lift
        if ((liftPower < 0) && (limitLowerLift.isPressed()))
        {
            motorLift.setPower(0);//don't run past limit switch
        }
        else //allow stick to control lift
        {
            motorLift.setPower(liftPower);//control lift with right stick
        }

        if (limitLowerLift.isPressed() && !limitLowerLiftPrevious)//rising edge reset
        {
            motorLift.setMode(DcMotorController.RunMode.RESET_ENCODERS);//may require delay
            telemetry.addData("LiftMode", "Lift encoders reset");
        }
        limitLowerLiftPrevious=limitLowerLift.isPressed();

        //check if finished resetting
        if ((motorLift.getMode()==DcMotorController.RunMode.RESET_ENCODERS)&&(motorLift.getCurrentPosition() == 0))
        {
            motorLift.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            telemetry.addData("LiftMode", "Lift switched to Run using encoders");
        }


        //Limit and Encoder Reset code for arm

        if ((armPower < 0) && (limitLowerArm.isPressed()))
        {
            motorArm.setPower(0);//don't run past limit switch
        }
        else //allow stick to control lift
        {
            motorArm.setPower(armPower);//control lift with right stick
        }

        if (limitLowerArm.isPressed() && !limitLowerArmPrevious)//rising edge
        {
            motorArm.setMode(DcMotorController.RunMode.RESET_ENCODERS);//may require delay
            telemetry.addData("ArmMode", "Arm encoders reset");
        }
        limitLowerArmPrevious=limitLowerArm.isPressed();

        //check if finished resetting
        if ((motorArm.getMode()==DcMotorController.RunMode.RESET_ENCODERS)&&(motorArm.getCurrentPosition() == 0))
        {
            motorArm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            telemetry.addData("ArmMode", "Arm switched to Run using encoders");
        }


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);


        if (SlowMode) {
            right/=2;
            left/=2;
        }


        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);

        // update the position of the paddle servos
        if (gamepad2.b) {
            paddleLeftPosition = LEFT_PADDLE_PRESS;
        }
        else
        {
            paddleLeftPosition = LEFT_PADDLE_RETRACT;
        }

        if (gamepad2.x) {

            paddleRightPosition = RIGHT_PADDLE_PRESS;
        }
        else
        {
            paddleRightPosition = RIGHT_PADDLE_RETRACT;
        }

        paddleRight.setPosition(paddleRightPosition);
        paddleLeft.setPosition(paddleLeftPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        //read gyro stuff
        heading = sensorGyro.getHeading();
        xVal = sensorGyro.rawX();
        yVal = sensorGyro.rawY();
        zVal = sensorGyro.rawZ();

        Color.RGBToHSV(sensorBeacon.red() * 8, sensorBeacon.green() * 8, sensorBeacon.blue() * 8, beaconhsvValues);
        Color.RGBToHSV(sensorLine.red() * 8, sensorLine.green() * 8, sensorLine.blue() * 8, linehsvValues);

        telemetry.addData("paddleRight", "paddleRight:  " + String.format("%.2f", paddleRightPosition));
        telemetry.addData("paddleLeft", "paddleLeft:  " + String.format("%.2f", paddleLeftPosition));
        telemetry.addData("left position",motorLeft.getCurrentPosition());
        telemetry.addData("right position",motorRight.getCurrentPosition());
        telemetry.addData("lift position", motorLift.getCurrentPosition());
        telemetry.addData("arm position", motorArm.getCurrentPosition());
        telemetry.addData("BeaconHSV", "BHSV: " + String.format(" %.3f %.3f %.3f ", beaconhsvValues[0], beaconhsvValues[1],beaconhsvValues[2]));
        telemetry.addData("LineHSV", "LHSV: " + String.format(" %.3f %.3f %.3f ", linehsvValues[0], linehsvValues[1],linehsvValues[2]));
        telemetry.addData("BeaconRGBC", "BeaconRBGC: "+ String.format("%d %d %d %d", sensorBeacon.red(),sensorBeacon.blue(),sensorBeacon.green(),sensorBeacon.alpha()));
        telemetry.addData("LineRGBC", "LineRBGC: " + String.format("%d %d %d %d", sensorLine.red(), sensorLine.blue(), sensorLine.green(), sensorLine.alpha()));
        telemetry.addData("Heading",heading);
        telemetry.addData("GyroRaw","GyroRaw: " + String.format("%d %d %d", xVal, yVal, zVal));
        telemetry.addData("SlowMode",SlowMode);
        telemetry.addData("limitLowerLift",limitLowerLift.isPressed());
        telemetry.addData("limitLowerArm",limitLowerArm.isPressed());
        telemetry.addData("liftBusy",motorLift.isBusy());
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

