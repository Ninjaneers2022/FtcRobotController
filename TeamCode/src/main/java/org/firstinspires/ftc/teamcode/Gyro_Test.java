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
        //robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);
//set power for all wheels indefinitely
        double power = .3;
        //put moves here

        telemetry.addData("Mode", "gyro calibrated...waiting for start");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        //robot.gyro.resetZAxisIntegrator();


        while (opModeIsActive())
        {
            telemetry.addData("gyro heading", robot.getHeading());
            telemetry.update();

            // Use gyro to drive in a straight line.
            double correction = checkDirection();

            // set power levels.
            robot.leftDrive.setPower(power - correction);
            robot.rightDrive.setPower(power + correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            if (gamepad1.a || gamepad1.b)
            {
                // backup.
                robot.leftDrive.setPower(power);
                robot.rightDrive.setPower(power);

                sleep(500);

                // stop.
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                // turn 90 degrees right.
                if (gamepad1.a) rotate(-90, power);

                // turn 90 degrees left.
                if (gamepad1.b) rotate(90, power);
            }
        }

        // turn the motors off.
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, heading, gain = .10;

        heading = robot.getHeading();

        if (heading == 0)
            correction = 0;             // no adjustment.
        else if (heading > 180)
            correction = 360 - heading; // adjust left.
        else
            correction = -heading;      // adjust right.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 350 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;
        int targetAngle;

        // reset gyro to zero.
        //robot.gyro.resetZAxisIntegrator();

        // Gyro returns 0->359 when rotating counter clockwise (left) and 359->0 when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
            targetAngle = 360 + degrees;    // degrees is - for right turn.
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
            targetAngle = degrees;
        } else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        for (int i = 0; i < 5000; i++)
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && robot.getHeading() == 0) {
                    telemetry.addData("gyro heading", robot.getHeading());
                    telemetry.update();
                    idle();
                }

                while (opModeIsActive() && robot.getHeading() > targetAngle) {
                    telemetry.addData("gyro heading", robot.getHeading());
                    telemetry.update();
                    idle();
                }
            } else
                while (opModeIsActive() && robot.getHeading() < targetAngle) {
                    telemetry.addData("gyro heading", robot.getHeading());
                    telemetry.update();
                    idle();
                }


        // turn the motors off.
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);

        // Reset gyro heading to zero on new direction we are now pointing.
        //robot.gyro.resetZAxisIntegrator();


    }
}