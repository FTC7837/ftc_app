package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class EncoderMotorTask {

    private String name;
    private DcMotor motor;
    private double power;
    private LinearOpMode opMode;

    private double targetEncoderValue;

    private boolean running = false;

    public static enum Direction {
        FORWARD,
        BACKWARD
    }
    public EncoderMotorTask(LinearOpMode opMode, DcMotor motor , String name) {

        this.opMode = opMode;
        this.motor = motor;
        this.name = name;

    }


    public void startMotor(double power,
                           double targetEncoderValue,
                           EncoderMotorTask.Direction direction) throws InterruptedException {

        running = true;
        this.targetEncoderValue = targetEncoderValue;

        double powerWithDirection = (direction == Direction.FORWARD) ? power : -power;

        resetEncoder();
        waitForEncoderToReset();

        motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        if (motor != null) {
            motor.setPower(powerWithDirection);
        }

        opMode.telemetry.addData("starting motor" , name);

    }

    public boolean targetReached() {

        boolean reached = false;

        if (motor != null)
        {
            int position = Math.abs (motor.getCurrentPosition ());
            opMode.telemetry.addData(name + "current position" , position);

            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (position > this.targetEncoderValue)
            {
                //
                // Set the status to a positive indication.
                //
                reached = true;
                opMode.telemetry.addData("Target Reached" , name);
            }
        }

        return reached;

    }


    public void stop()  throws InterruptedException {
        running = false;
        this.motor.setPower(0.0f);

    }

    public boolean isRunning() {
        return running;
    }



    private void resetEncoder () throws InterruptedException {
        //
        // Reset the motor encoders on the drive wheels.
        //
        if (motor != null) {
            motor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        }

    }

    private void waitForEncoderToReset() throws InterruptedException {

        while(motor.getCurrentPosition() != 0 ){
            opMode.waitForNextHardwareCycle();
        }

    }



}