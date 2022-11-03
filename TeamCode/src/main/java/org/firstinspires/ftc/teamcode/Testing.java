package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Testing extends  LinearOpMode{
    Ninjabot robot;

    final int FORWARD = 1;
    final int BACKWARD = 3;
    final int ROTATE_LEFT = 5;
    final int ROTATE_RIGHT = 6;
    final int TANK_LEFT= 7;
    final int TANK_RIGHT= 8;
    final int clawOpen = 1;
    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

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

        waitForStart();
        robot.leftDrive.setPower(0.4);
        robot.rightDrive.setPower(0.4);
        robot.liftMotor.setPower(1);

        //liftSetPosition(1);
        int liftPosition = 6500;
        robot.liftMotor.setTargetPosition(liftPosition);
        while(!robot.inRange(robot.liftMotor.getCurrentPosition(),liftPosition,100) && opModeIsActive()){
            sleep(200);
        }
        robot.driveTo(100,FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.liftMotor.setTargetPosition(liftPosition - 1000);
        while(robot.inRange(robot.liftMotor.getCurrentPosition(),(liftPosition - 1000),100) && opModeIsActive()){
            sleep(200);
        }
        robot.claw.setPosition(1);
        robot.liftMotor.setTargetPosition(liftPosition);
        while(!robot.inRange(robot.liftMotor.getCurrentPosition(),liftPosition,100) && opModeIsActive()){
            sleep(200);
        }
        robot.driveTo(100, BACKWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();







        /*double startH = robot.getHeading();
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

         */
    }
    public void liftSetPosition(int level){
        int liftposition;
        if (level == 1){
            liftposition = 6000;
        }
        else{
            liftposition = 8000;
        }
        deposit(liftposition);
    }
    public void deposit(int liftPosition){
        robot.liftMotor.setTargetPosition(liftPosition);
        while(robot.liftMotor.getCurrentPosition() != liftPosition){
            sleep(200);
        }
        //robot.driveTo(100,FORWARD);
        //while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.liftMotor.setTargetPosition(liftPosition - 1000);
        while(robot.liftMotor.getCurrentPosition() != (liftPosition - 1000)){
            sleep(200);
        }
        robot.claw.setPosition(1);
        robot.liftMotor.setTargetPosition(liftPosition);
        while(robot.liftMotor.getCurrentPosition() != liftPosition){
            sleep(200);
        }
        robot.driveTo(100, BACKWARD);
    }
}

