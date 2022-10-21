/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/*
 * This sample demonstrates a basic sleeve position
 */
//@TeleOp
@Autonomous
public class AutoSignal2 extends LinearOpMode {
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
        
      //  robot.gyroCalibrate();

        //while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addData( "AVG1", robot.sleeveNumber.ColourAvg()  ); telemetry.update();

      //  }
		// Wait for the game to start
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

        robot.leftDrive.setPower(0.4);
        robot.rightDrive.setPower(0.4);

        // Move arm to signal delivery position (out of webcam view) while driving forward
        // Dive forward X inches

        robot.driveTo(550, FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //up

        robot.liftMotor.setTargetPosition(0 + 20);
        robot.liftMotor.setPower(.1);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move wrist 90 degrees .25
        robot.wrist.setPosition(-0.25);

        // lower the lift arm
        robot.liftMotor.setTargetPosition(0 - 30);
        robot.liftMotor.setPower(-.1);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Open Claw slowly
        robot.wrist.setPosition(-0.1);
        robot.wrist.setPosition(-0.1);
        robot.wrist.setPosition(-0.1);

        //raise lift arm
        robot.liftMotor.setTargetPosition(0 + 50);
        robot.liftMotor.setPower(.1);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.driveTo(100, BACKWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(550, ROTATE_LEFT); //950 is equal to a 180 degree turn of the robot in rotate turns
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(100, FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(550, ROTATE_RIGHT);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        robot.driveTo(100, FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

    }
    }
