package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

@Autonomous
public class Testing extends  LinearOpMode{
    Ninjabot robot;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);
        //GyroSensor gyro;
        //gyro = hardwareMap.gyroSensor.get("gyro");
        telemetry.addData("Mode", "starting gyro calibration...please wait");
        telemetry.update();

        //gyro.calibrate();

        int FORWARD = 1;
        int BACKWARD = 3;
        int ROTATE_LEFT = 5;
        int ROTATE_RIGHT = 6;
        int TANK_LEFT= 7;
        int TANK_RIGHT= 8;


        waitForStart();
        telemetry.addData("Status", "ClawStatus");
        telemetry.update();
        //robot.claw.setPosition(0);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftDrive.setPower(0.4);
        robot.rightDrive.setPower(0.4);
//set power for all wheels indefinitely

        //Put moves here

        robot.driveTo(550, ROTATE_LEFT); //950 is equal to a 180 degree turn of the robot in rotate turns
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        sleep(1000);

        robot.driveTo(robot.convert(50), FORWARD); //drive backward towards the duck carosel
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.leftDrive.setPower(50);
        sleep(2000);
        robot.leftDrive.setPower(0);

        robot.claw.setPosition(50);
        robot.claw.setPosition(40);
        robot.claw.setPosition(50);

    }

}