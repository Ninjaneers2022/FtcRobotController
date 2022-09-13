package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class automated_duck extends LinearOpMode{
    Ninjabot robot;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        int FORWARD = 1;
        int BACKWARD = 3;
        int ROTATE_LEFT = 5;
        int ROTATE_RIGHT = 6;
        //int SPIN = 4;

        Ninjabot robot;
        robot = new Ninjabot(hardwareMap, this);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        telemetry.addData("Status", "ClawStatus");
        telemetry.update();
        robot.claw.setPosition(1);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
        robot.spinner.setTargetPosition(0);
// zero out the motors counters
        robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (!opModeIsActive());

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);
        robot.spinner.setPower(0.5);

        //Put moves here

        //robot.driveTo(robot.convert(10), FORWARD);
        //while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

            //robot.driveTo(950, ROTATE_LEFT); //180 degree turn
        //while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //activate spinner

      //  sleep(2000);

        //robot.liftArm.setTargetPosition(20);
        //robot.liftArm.setPower(0.5);
        //robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);

        robot.driveTo(robot.convert(-15), FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        //while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //robot.driveTo(robot.convert(2), ROTATE_RIGHT);
    }

    }
