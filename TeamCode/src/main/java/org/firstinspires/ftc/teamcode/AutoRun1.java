

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
        //robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        robot.leftDrive.setPower(0.4);
        robot.rightDrive.setPower(0.4);


        int yellow = 75; // sleeve 2
        int red = 100; // sleeve 1
        int blue = 150; // sleeve 3

        if (robot.inRange(position, red, 10)){
            // move forward a tad
            robot.driveTo(50, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn left 90 degrees
            robot.driveTo(550, ROTATE_LEFT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward one square
            robot.driveTo(250, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // turn 90 degrees right
            robot.driveTo(550, ROTATE_RIGHT);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
            // move forward into the section
            robot.driveTo(600, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        }
        if (robot.inRange(position, yellow, 10)){
            robot.driveTo(600, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
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
            robot.driveTo(600, BACKWARD);
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        }


    }
}
