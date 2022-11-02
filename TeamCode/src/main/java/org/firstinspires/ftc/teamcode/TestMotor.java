package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class TestMotor extends  LinearOpMode{
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

        //test drive base
        robot.driveTo(500, FORWARD);
        telemetry.addData("robot move", "forward");
        telemetry.update();
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(500, BACKWARD);
        telemetry.addData("robot move", "backward");
        telemetry.update();
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(500, ROTATE_LEFT);
        telemetry.addData("robot move", "left");
        telemetry.update();
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(500, ROTATE_RIGHT);
        telemetry.addData("robot move", "right");
        telemetry.update();
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //test lift system
        robot.liftMotor.setTargetPosition(5000);
        telemetry.addData("raise", "lift");
        telemetry.update();
        while (robot.inRange(robot.liftMotor.getCurrentPosition(), 5000, 100)) {
            sleep(200);
        }
        robot.liftMotor.setTargetPosition(0);
        telemetry.addData("lower", "lift");
        telemetry.update();
        while (robot.inRange(robot.liftMotor.getCurrentPosition(), 0, 100)) {
            sleep(200);
        }
        //test wrist
        robot.wrist.setPosition(0.4);
        telemetry.addData("open", "wrist");
        telemetry.update();
        while (robot.wrist.getPosition() == 0.4) {
            sleep(200);
        }
        robot.wrist.setPosition(0.8);
        telemetry.addData("close", "wrist");
        telemetry.update();
        while (robot.wrist.getPosition() == 0.8) {
            sleep(200);
        }
        //test claw
        robot.claw.setPosition(0.4);
        telemetry.addData("open", "claw");
        telemetry.update();
        while (robot.wrist.getPosition() == 0.4) {
            sleep(200);
        }
        robot.claw.setPosition(0.8);
        telemetry.addData("close", "claw");
        telemetry.update();
        while (robot.wrist.getPosition() == 0.8) {
            sleep(200);
        }
    }
}