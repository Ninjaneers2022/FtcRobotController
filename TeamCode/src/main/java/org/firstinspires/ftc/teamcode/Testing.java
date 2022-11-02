package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Testing extends  LinearOpMode{
    Ninjabot robot;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        final int FORWARD = 1;
        final int BACKWARD = 3;
        final int ROTATE_LEFT = 5;
        final int ROTATE_RIGHT = 6;
        final int TANK_LEFT= 7;
        final int TANK_RIGHT= 8;
        final int clawOpen = 1;



        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
        robot.liftMotor.setTargetPosition(0);
// zero out the motors counters

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double startH = robot.getHeading();
        telemetry.addData("starting gyro heading", startH);
        telemetry.update();

        waitForStart();
        robot.leftDrive.setPower(0.4);
        robot.rightDrive.setPower(0.4);


        sleep(1000);

        robot.driveTo(800, ROTATE_LEFT); //turn to 90
        while (!robot.targetReached() && opModeIsActive()){
            telemetry.addData("after turn one heading is", robot.getHeading());
            telemetry.update();
        }
        sleep(1000);
        double HOne = robot.getHeading();
        sleep(1000);


        robot.driveTo(800, ROTATE_RIGHT); //turn to 90
        while (!robot.targetReached() && opModeIsActive()){
            telemetry.addData("after turn two heading is", robot.getHeading());
            telemetry.update();
        }

        sleep(1000);
        double HTwo = robot.getHeading();
        sleep(1000);

        double distance = HOne - HTwo;
        telemetry.addData("1.....", HOne);
        telemetry.addData("2.....", HTwo);
        telemetry.addData("3.....", distance);
        telemetry.update();
        sleep(100000);
    }

}