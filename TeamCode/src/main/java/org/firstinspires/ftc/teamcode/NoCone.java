

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import javax.tools.ForwardingFileObject;


/*
 * This sample demonstrates a basic sleeve position
 */
//@TeleOp
@Autonomous
@Disabled
public class NoCone extends LinearOpMode {
    Ninjabot robot;

    private double cAngle ;
    private double startAngle ;
    private double finishAngle ;
    private double result ;

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

        //  robot.gyroCalibrate();

        while (!opModeIsActive() && !isStopRequested()) {
            int position = robot.sleeveNumber.ColourAvg();
            telemetry.addData( "AVG1", position  ); telemetry.update();
            telemetry.update();
        }
        // Wait for the game to start

        int position = robot.sleeveNumber.ColourAvg();

        //int position = 179;

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

        int yellow = 85; // sleeve 2// at home number is 80
        int red = 105; // sleeve 1
        int blue = 175; // sleeve 3 //150 at home

        //coment out if you are actually using the vision code
        //position = blue;

        if (robot.inRange(position, blue, 10)){
            // move forward a tad
            robot.driveTo(200, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn right 90 degrees
            gyroMathturn(90);
            //robot.driveTo(460, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward one square
            robot.driveTo(1300, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn left 90 degrees
            gyroMathturn(-90);
            //robot.driveTo(380, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward into the section
            robot.driveTo(1300, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            telemetry.addData("Position", "red");
            telemetry.update();
        }
        if (robot.inRange(position, red, 8)){
            // move forward a tad
            telemetry.addData("new", "code");
            telemetry.update();
            robot.driveTo(300, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn right 90 degrees
            gyroMathturn(-90);
            //robot.driveTo(400, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward one square
            robot.driveTo(1000, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn left 90 degrees
            gyroMathturn(90);
            //robot.driveTo(380, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward into the section
            robot.driveTo(1300, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            telemetry.addData("Position", "blue");
            telemetry.update();
        }
        if (robot.inRange(position, yellow, 10)){
            robot.driveTo(1500, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        }


    }
// positive = left turn, negative = right turn
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
