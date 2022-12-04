package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
// out of date
public class RRun extends LinearOpMode{
    Ninjabot robot;

    double startAngle ;
    double finishAngle ;
    double result ;
    int FORWARD = 1;
    int BACKWARD = 3;
    int ROTATE_LEFT = 5;
    int ROTATE_RIGHT = 6;
    int TANK_LEFT= 7;
    int TANK_RIGHT= 8;

    int LeftWheel;
    int RightWheel;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        //robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        //robot.claw.setPosition(0);

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

        while (!opModeIsActive());

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);


//set power for all wheels indefinitely
        //Put moves here
/*
        LeftWheel = RightWheel = 325;
        robot.leftDrive.setTargetPosition(LeftWheel);
        robot.rightDrive.setTargetPosition(RightWheel);
        while(robot.leftDrive.getCurrentPosition() != LeftWheel && robot.rightDrive.getCurrentPosition() != RightWheel && opModeIsActive()){
            sleep(200);
        }

        gyroMathturn(90);

        LeftWheel = robot.leftDrive.getCurrentPosition() + 620;
        RightWheel = robot.rightDrive.getCurrentPosition() + 620;
        robot.leftDrive.setTargetPosition(LeftWheel);
        robot.rightDrive.setTargetPosition(RightWheel);
        while(robot.leftDrive.getCurrentPosition() != LeftWheel && robot.rightDrive.getCurrentPosition() != RightWheel && opModeIsActive()){
            sleep(200);
        }


 */
        leftGyroMathturn(-90);

        rightGyroMathturn(90);

        sleep(2000);
/*
        LeftWheel = robot.leftDrive.getCurrentPosition() + 400;
        RightWheel = robot.rightDrive.getCurrentPosition() + 400;
        robot.leftDrive.setTargetPosition(LeftWheel);
        robot.rightDrive.setTargetPosition(RightWheel);
        while(robot.leftDrive.getCurrentPosition() != LeftWheel && robot.rightDrive.getCurrentPosition() != RightWheel && opModeIsActive()){
            sleep(200);
        }
        gyroMathturn(-45);


 */

    }

    public void rightGyroMathturn(int bearing){
        int intialTurn = (int)((460/90)* bearing);
        startAngle = robot.getAngle();
        telemetry.addData("start",startAngle);
        result = (startAngle + bearing) - Math.abs(robot.getAngle());
        telemetry.addData(" 0ff result",result);
        telemetry.update();
        robot.driveTo(intialTurn, ROTATE_RIGHT);
        while (!robot.targetReached() && opModeIsActive()) sleep(200);
        for (int i = 0; i < 3; i++) {
            if (startAngle + bearing - robot.getAngle() > 10)
                robot.driveTo(50, ROTATE_RIGHT);
            else if (startAngle + bearing - robot.getAngle() > 5)
                robot.driveTo(20, ROTATE_RIGHT);
            else if (startAngle + bearing - robot.getAngle() > 3)
                robot.driveTo(10, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) sleep(200);}
        finishAngle = Math.abs(robot.getAngle());
        telemetry.update();
    }
    public void leftGyroMathturn(int bearing){
        int intialTurn = (int)((460/90)* bearing);
        startAngle = robot.getAngle();
        telemetry.addData("start",startAngle);
        result = (startAngle + bearing) - Math.abs(robot.getAngle());
        telemetry.addData(" 0ff result",result);
        telemetry.update();
        robot.driveTo(intialTurn, ROTATE_LEFT);
        while (!robot.targetReached() && opModeIsActive()) sleep(200);
        for (int i = 0; i < 3; i++) {
            if (startAngle + bearing - robot.getAngle() > 10)
                robot.driveTo(50, ROTATE_LEFT);
            else if (startAngle + bearing - robot.getAngle() > 5)
                robot.driveTo(20, ROTATE_LEFT);
            else if (startAngle + bearing - robot.getAngle() > 3)
                robot.driveTo(10, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) sleep(200);}
        finishAngle = Math.abs(robot.getAngle());
        telemetry.update();
    }
}


