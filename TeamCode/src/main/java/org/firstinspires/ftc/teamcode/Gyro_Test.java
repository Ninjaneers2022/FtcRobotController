package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Gyro_Test extends LinearOpMode {
    Ninjabot robot;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        waitForStart();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);
//set power for all wheels indefinitely
        //put moves here
        robot.turn(10);
        sleep(9000);
        robot.turn(90);
        sleep(9000);
        robot.gyroTurn(50, -135);
        sleep(9000);


    }
}