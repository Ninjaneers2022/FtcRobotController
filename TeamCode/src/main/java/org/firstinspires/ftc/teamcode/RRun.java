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
        startAngle = robot.getAngle();

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

        waitForStart();

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);


//set power for all wheels indefinitely
        //Put moves here
        LeftWheel = RightWheel = 325;
        robot.leftDrive.setTargetPosition(LeftWheel);
        robot.rightDrive.setTargetPosition(RightWheel);
        while(robot.leftDrive.getCurrentPosition() != LeftWheel && robot.rightDrive.getCurrentPosition() != RightWheel && opModeIsActive()){
            sleep(200);
        }

        LeftGyroMathturn(-90);

        LeftWheel = robot.leftDrive.getCurrentPosition() + 620;
        RightWheel = robot.rightDrive.getCurrentPosition() + 620;
        robot.leftDrive.setTargetPosition(LeftWheel);
        robot.rightDrive.setTargetPosition(RightWheel);
        while(robot.leftDrive.getCurrentPosition() != LeftWheel && robot.rightDrive.getCurrentPosition() != RightWheel && opModeIsActive()){
            sleep(200);
        }

        LeftGyroMathturn(-90);

        sleep(2000);

        LeftWheel = robot.leftDrive.getCurrentPosition() + 400;
        RightWheel = robot.rightDrive.getCurrentPosition() + 400;
        robot.leftDrive.setTargetPosition(LeftWheel);
        robot.rightDrive.setTargetPosition(RightWheel);
        while(robot.leftDrive.getCurrentPosition() != LeftWheel && robot.rightDrive.getCurrentPosition() != RightWheel && opModeIsActive()){
            sleep(200);
        }
        LeftGyroMathturn(-45);
    }

    public int turnMath(int degree){
        int turn = (int)((460/90)* (degree - robot.getAngle()));
        return turn;
    }
    public void LeftGyroMathturn(int bearing){
        int intialTurn = (int)((460/90)* (bearing - robot.getAngle()));
        robot.driveTo(intialTurn, ROTATE_LEFT);
        while (!robot.targetReached() && opModeIsActive()) sleep(200);
        int test = (int)(bearing - robot.getAngle());
        for (int i = 0; i < 3; i++) {
            if (test > 10)
                robot.driveTo(50, ROTATE_LEFT);
            else if (test> 5)
                robot.driveTo(20, ROTATE_LEFT);
            else if ( test > 3)
                robot.driveTo(10, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) sleep(200);
        }
    }
    public void RightGyroMathturn(int bearing){
        int intialTurn = (int)((460/90)* (bearing - robot.getAngle()));
        robot.driveTo(intialTurn, ROTATE_RIGHT);
        while (!robot.targetReached() && opModeIsActive()) sleep(200);
        int test = (int)(bearing - robot.getAngle());
        for (int i = 0; i < 3; i++) {
            if (test > 10)
                robot.driveTo(50, ROTATE_RIGHT);
            else if (test> 5)
                robot.driveTo(20, ROTATE_RIGHT);
            else if ( test > 3)
                robot.driveTo(10, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) sleep(200);
        }
    }
}


