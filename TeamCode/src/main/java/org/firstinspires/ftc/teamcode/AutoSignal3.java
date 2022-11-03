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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/*
 * This sample demonstrates a basic sleeve position
 */
//@TeleOp
@Autonomous
@Disabled
public class AutoSignal3 extends LinearOpMode {
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


    
        robot.wrist.setPosition(0.4); //;out
    }
    }
