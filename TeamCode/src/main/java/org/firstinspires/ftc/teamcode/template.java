package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
// out of date
public class template extends LinearOpMode{
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

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        //robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addData("Status", "ClawStatus");
        telemetry.update();
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
/*
        telemetry.addData("testing", "123");
        telemetry.update();
        robot.claw.setPosition(0.5);
        while (robot.claw.getPosition() != 0.5 && opModeIsActive()){
            telemetry.addData("Claw","closing");
            telemetry.addData("Claw",robot.claw.getPosition());
            telemetry.update();
        }


 */
        while (!opModeIsActive());

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);


//set power for all wheels indefinitely
        //Put moves here
/*
        sleep(200);

        robot.liftMotor.setPower(1);

        robot.liftMotor.setTargetPosition(-150);
        while (robot.liftMotor.getCurrentPosition() != -150 && opModeIsActive()){
            telemetry.addData("Lifting","now");
            telemetry.addData("Lift",robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.rightElbow.setPosition(0.7);
        robot.leftElbow.setPosition(0.3);
        while (robot.rightElbow.getPosition() != 0.7 && opModeIsActive()){
            telemetry.addData("Lifting Elbow","now");
            telemetry.addData("Right Elbow", robot.rightElbow.getPosition());
            telemetry.addData("Left Elbow", robot.rightElbow.getPosition());
            telemetry.update();
        }


 */

        //2371
        //2160
        robot.leftDrive.setTargetPosition(325);
        robot.rightDrive.setTargetPosition(325);
        //2142
        //2923
        while(robot.leftDrive.getCurrentPosition() != 325 && robot.rightDrive.getCurrentPosition() != 325 && opModeIsActive()){
            sleep(200);
        }

        gyroMathturn(90);
        //2701
        //2471
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + 620);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + + 620);
        while(robot.leftDrive.getCurrentPosition() != 325 && robot.rightDrive.getCurrentPosition() != 325 && opModeIsActive()){
            sleep(200);
        }

        //3321
        //4086

        }

    public void gyroMathturn(int bearing){
        int intialTurn = (int)((460/90)* bearing);
        startAngle = robot.getAngle() ;
        telemetry.addData("start",startAngle) ;
        finishAngle = robot.getAngle() ;
        finishAngle = Math.abs(finishAngle) ;
        telemetry.addData("finish",finishAngle) ;
        result = startAngle + bearing - finishAngle ;
        telemetry.addData("result",result) ;
        telemetry.update();
        if (bearing >= 0) {
            robot.driveTo(intialTurn, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) sleep(200);
            for (int i = 0; i < 3; i++) {
                if (startAngle + bearing - finishAngle > 10)
                    robot.driveTo(50, ROTATE_LEFT);
                else if (startAngle + bearing - finishAngle > 5)
                    robot.driveTo(20, ROTATE_LEFT);
                else if (startAngle + bearing - finishAngle > 3) ;
                robot.driveTo(10, ROTATE_LEFT);
                while (!robot.targetReached() && opModeIsActive()) sleep(200);
            }
        }
        else {
            robot.driveTo(intialTurn, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) sleep(200);
            for (int i = 0; i < 3; i++) {
                if (startAngle + bearing - finishAngle > 10)
                    robot.driveTo(50, ROTATE_LEFT);
                else if (startAngle + bearing - finishAngle > 5)
                    robot.driveTo(20, ROTATE_LEFT);
                else if (startAngle + bearing - finishAngle > 3) ;
                robot.driveTo(10, ROTATE_LEFT);
                while (!robot.targetReached() && opModeIsActive()) sleep(200);
            }
        }
    }
    }


