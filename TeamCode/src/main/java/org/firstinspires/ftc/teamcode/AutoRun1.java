

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import javax.tools.ForwardingFileObject;


/*
 * This sample demonstrates a basic sleeve position
 */
//@TeleOp
@Autonomous
public class AutoRun1 extends LinearOpMode {
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

        //  robot.gyroCalibrate();

        while (!opModeIsActive() && !isStopRequested()) {
            int position = robot.sleeveNumber.ColourAvg();
            telemetry.addData( "AVG1", position  ); telemetry.update();
            telemetry.update();
        }
        // Wait for the game to start

        int position = robot.sleeveNumber.ColourAvg();


        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        robot.leftDrive.setPower(0.4);
        robot.rightDrive.setPower(0.4);


        int yellow = 80; // sleeve 2
        int red = 105; // sleeve 1
        int blue = 150; // sleeve 3

        if (robot.inRange(position, red, 10)){
            // move forward a tad
            robot.driveTo(50, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn left 90 degrees
            robot.driveTo(550, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward one square
            robot.driveTo(250, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn 90 degrees right
            robot.driveTo(550, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward into the section
            robot.driveTo(1200, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            telemetry.addData("Position", "red");
            telemetry.update();
        }
        if (robot.inRange(position, yellow, 10)){
            robot.driveTo(100, FORWARD);
            telemetry.addData("move:" ,"forward 1");
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            robot.driveTo(275, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            telemetry.addData("move:" ,"left");
            //while (robot.inRange(robot.getHeading(),89,5));
            /*robot.liftMotor.setTargetPosition(8000);
            telemetry.addData("move:" ,"up");
            robot.driveTo(600, FORWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            telemetry.addData("move:" ,"forward 2");
            telemetry.addData("Position", "yellow");
            telemetry.update();
            sleep(20000);

             */
        }
        if (robot.inRange(position, blue, 10)){
            // move forward a tad
            robot.driveTo(50, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn right 90 degrees
            robot.driveTo(550, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward one square
            robot.driveTo(250, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn left 90 degrees
            robot.driveTo(550, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward into the section
            robot.driveTo(1200, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            telemetry.addData("Position", "blue");
            telemetry.update();
        }


    }
}
