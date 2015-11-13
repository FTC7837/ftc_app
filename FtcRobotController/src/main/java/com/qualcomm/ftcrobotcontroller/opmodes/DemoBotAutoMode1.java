/*
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class DemoBotAutoMode1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        int step = 1;
        EncoderMotorTask motorTask = new EncoderMotorTask( this, motor1FromHardwareMap ,"motor1");

        while (opModeIsActive()) {

            if (kill-switch-sensor-ispressed) {
                if ( motorTask.isRunning() ) {
                    motorTask.stop();
                }
                step = 99;
            }

            switch (step) {
                case 1:
                    if (! motorTask.isRunning()) {
                        //full power , forward for 2880
                        motorTask.startMotor(1, 2880 , EncoderMotorTask.Direction.FORWARD);
                    }
                    if (motorTask.targetReached()) {
                        motorTask.stop();
                        step++;
                    }
                    break;

                case 2:
                    if (! motorTask.isRunning()) {
                        //  1/4 power backward for 1000
                        motorTask.startMotor(0.25, 1000 , EncoderMotorTask.Direction.BACKWARD);
                    }
                    if (motorTask.targetReached()) {
                        motorTask.stop();
                        step++;
                    }
                    break;

                case 99:
                    telemetry.addData("Opmode Status" , "Robot Stopped.  Kill switch activated");
                    break;

                default:
                    telemetry.addData("Opmode Status" , "Tasks completed");
                    break;

            }
            waitOneFullHardwareCycle();
        }
    }
}
*/