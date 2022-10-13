package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class Auto_Rings extends LinearOpMode
{
    HardwareGracebot robot;

    @Override
    public void runOpMode()
    {
        robot = new HardwareGracebot(hardwareMap, this );

        telemetry.addData( "ringFinder", "started"  ); telemetry.update();
        robot.gyroCalibrate();

        // Wait for the game to start
        waitForStart();

        for ( double value : new double [] { 1.00, 1.75, 2.0, 2.25, 2.5 } )
        {
            robot.ringFinder.magnify( value );
            // check number of rings
            telemetry.addData("magnification:", value );
            switch( robot.ringFinder.count())
            {
                case -1 : telemetry.addData( "ringFinder:", "broken"  ); break; // broken
                case  0 : telemetry.addData( "ringFinder:", "none"    ); break; // none in sight - A
                case  1 : telemetry.addData( "ringFinder:", "one"     ); break; // 1 ring found - B
                case  4 : telemetry.addData( "ringFinder:", "four"    ); break; // 4 rings found - C
                default : telemetry.addData( "ringFinder:", "unknown" ); break;
            }
            telemetry.addData( "confidence:", robot.ringFinder.connum   );
            telemetry.update();

            sleep(5 * 1000 );
        }
    }  // runOpMode

    public void updateWheelTelemetry()
    {
        telemetry.addData("f_right", robot.f_right.getCurrentPosition());
        telemetry.addData("f_left",  robot.f_left.getCurrentPosition());
        telemetry.addData("b_right", robot.b_right.getCurrentPosition());
        telemetry.addData("b_left",  robot.b_left.getCurrentPosition());
        telemetry.update();
    }
} // end of class
