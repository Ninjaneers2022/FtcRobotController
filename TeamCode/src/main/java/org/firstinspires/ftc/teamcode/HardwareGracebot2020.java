/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Right  drive motor:        "f_right"
 * Motor channel:  Right  drive motor:        "b_right"
 * Motor channel:  Left drive motor:        "f_left"
 * Motor channel:  Left drive motor:        "b_left"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareGracebot
{
    //    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device
    BNO055IMU             gyro    = null;
    Orientation           gyroLastAngles = null;
    double                gyroGlobalAngle;

    /* Public OpMode members. */
    public DcMotor  f_right   = null;
    public DcMotor  b_right  = null;
    public DcMotor  f_left   = null;
    public DcMotor  b_left   = null;
    public DcMotor  ARM_BASE = null;
    public Servo    ELBO     = null;
    public Servo    HAND   = null;
    public Servo    ELBOW2     = null;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final int FORWARD      = 1;
    public static final int RIGHT        = 2;
    public static final int BACKWARD     = 3;
    public static final int LEFT         = 4;
    public static final int ROTATE_LEFT  = 5;
    public static final int ROTATE_RIGHT = 6;

    static final int REV_ROBOTICS_HDHEX_MOTOR   = 28; // ticks per rotation
    static final int REV_ROBOTICS_HDHEX_20_to_1 = REV_ROBOTICS_HDHEX_MOTOR * 20;

    static final int DRIVE_MOTOR_TICK_COUNTS    = REV_ROBOTICS_HDHEX_20_to_1;
    static final double WHEEL_DIAMETER          = 4.0;

    static final int REV_ROBOTICS_COREHEX_MOTOR   = 4; // ticks per rotation
    static final int REV_ROBOTICS_COREHEX_72_to_1 = REV_ROBOTICS_COREHEX_MOTOR * 72;
    static final int ArmCounts                    = REV_ROBOTICS_COREHEX_72_to_1;


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.0125;    // Larger is more responsive, but also less stable

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    LinearOpMode control        =  null;
    private ElapsedTime period  = new ElapsedTime();

    public RingFinder  ringFinder = null;

    /* Constructor */
    public HardwareGracebot(HardwareMap map, LinearOpMode ctrl)
    {
        init(map, ctrl);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode ctrl )
    {
        // Save reference to Hardware map
        hwMap   = ahwMap;
        control = ctrl;
        ringFinder = new RingFinder();

        //Define and Initialize Motors
        f_right = hwMap.get(DcMotor.class, "f_right");
        b_right = hwMap.get(DcMotor.class, "b_right");
        f_left  = hwMap.get(DcMotor.class, "f_left");
        b_left  = hwMap.get(DcMotor.class, "b_left");
        ARM_BASE= hwMap.get(DcMotor.class, "ARM_BASE");
        HAND= hwMap.get(Servo.class, "HAND");

        ELBOW2= hwMap.get(Servo.class, "ELBOW2");
        ELBO= hwMap.get(Servo.class, "ELBO");

        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        f_right.setDirection(DcMotor.Direction.REVERSE); // 0 Set to FORWARD if using AndyMark motors
        f_left.setDirection(DcMotor.Direction.FORWARD);  // 1 Set to FORWARD if using AndyMark motors
        b_right.setDirection(DcMotor.Direction.FORWARD); // 3 Set to REVERSE if using AndyMark motors
        b_left.setDirection(DcMotor.Direction.FORWARD);  // 2 Set to FORWARD if using AndyMark motors

        // leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        //ARM_BASE = ;
        // Set all wheel motors to zero power
        setDriveMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0.0 );

        //leftArm.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hwMap.get( BNO055IMU.class, "imu");
        gyroLastAngles = new Orientation();
        gyroGlobalAngle = 0.0;

        //ringFinder.initialize(hwMap, ctrl, "Webcam 1");
    }

    public void setDriveTargets( int fr_dist, int fl_dist, int br_dist, int bl_dist )
    {
        f_right.setTargetPosition( f_right.getCurrentPosition() + fr_dist );
        f_left.setTargetPosition(  f_left.getCurrentPosition()  + fl_dist );
        b_right.setTargetPosition( b_right.getCurrentPosition() + br_dist );
        b_left.setTargetPosition(  b_left.getCurrentPosition()  + bl_dist );
    }

    public void driveTo(double distance,int dir)
    {
        int clicks = driveInchesToClicks( distance );

        switch( dir )
        {                     // displacements: f_right    f_left     b_right    b_left
            case FORWARD     : setDriveTargets( +clicks, +clicks, +clicks, +clicks); break;
            case BACKWARD    : setDriveTargets( -clicks, -clicks, -clicks, -clicks); break;
            case LEFT        : setDriveTargets( +clicks, -clicks, -clicks, +clicks); break;
            case RIGHT       : setDriveTargets( -clicks, +clicks, +clicks, -clicks); break;
            case ROTATE_LEFT : setDriveTargets( -clicks, +clicks, -clicks, +clicks); break;
            case ROTATE_RIGHT: setDriveTargets( +clicks, -clicks, +clicks, -clicks); break;
        }
    }

    public boolean targetReached()
    {   // get the average distance left to travel of all wheels (in ticks)
        int average = Math.abs(f_right.getTargetPosition() - f_right.getCurrentPosition());
        average += Math.abs(f_left.getTargetPosition() - f_left.getCurrentPosition());
        average += Math.abs(b_right.getTargetPosition() - b_right.getCurrentPosition());
        average += Math.abs(b_left.getTargetPosition() - b_left.getCurrentPosition());
        average = average / 4;
        // must be 'reached' if less than 50
        return (average < 50);
        /*return inTolerance(m1.getCurrentPosition(), m1.getTargetPosition(), 60)
        && inTolerance(m2.getCurrentPosition(), m2.getTargetPosition(), 60)
        && inTolerance(m3.getCurrentPosition(), m3.getTargetPosition(), 60)
        && inTolerance(m4.getCurrentPosition(), m4.getTargetPosition(), 60);*/
    }

    public boolean driveIsBusy()
    {
        return( f_right.isBusy() || b_right.isBusy() || f_left.isBusy() || b_left.isBusy());
    }


    public void setDriveMode( DcMotor.RunMode mode, double power )
    {
        f_right.setPower( power ); f_right.setMode( mode );
        b_right.setPower( power ); b_right.setMode( mode );
        f_left.setPower(  power ); f_left.setMode(  mode );
        b_left.setPower(  power ); b_left.setMode(  mode );
    }

    public void setDriveMode( DcMotor.RunMode mode, double frightPower, double fleftPower, double brightPower,  double bleftPower)
    {
        f_right.setPower( frightPower ); f_right.setMode( mode );
        b_right.setPower( brightPower ); b_right.setMode( mode );
        f_left.setPower(  fleftPower  ); f_left.setMode(  mode );
        b_left.setPower(  bleftPower  ); b_left.setMode(  mode );
    }

    public void driveSetPower( double leftPower, double rightPower )
    {
        f_right.setPower( rightPower );
        b_right.setPower( rightPower );
        f_left.setPower(  leftPower );
        b_left.setPower(  leftPower );
    }

    public void driveSet4Power(  double frightPower, double fleftPower, double brightPower,  double bleftPower )
    {
        f_right.setPower( frightPower );
        b_right.setPower( brightPower );
        f_left.setPower(  fleftPower );
        b_left.setPower(  bleftPower );
    }

    public int driveInchesToClicks( double dist )
    {
        // the distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference   = 3.14 * WHEEL_DIAMETER;  //pi * diameter
        double rotationsNeeded = dist / circumference;

        return( (int) (rotationsNeeded * DRIVE_MOTOR_TICK_COUNTS ));
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle     )
    {
        // Ensure that the opmode is still active
        if (control.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            int clicks = driveInchesToClicks( distance );

            setDriveTargets( clicks, clicks, clicks, clicks );

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setDriveMode( DcMotor.RunMode.RUN_TO_POSITION, speed );

            // keep looping while we are still active, and BOTH motors are running.
            while (control.opModeIsActive() && driveIsBusy())
            {
                // adjust relative speed based on heading error.
                double error = angle - gyroGetAngle();
                double steer = error * P_DRIVE_COEFF;

                if (distance < 0)
                    steer *= -1.0;

                double leftSpeed  = speed - steer;
                double rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                leftSpeed  = Range.clip( leftSpeed,  -1.0, +1.0 );
                rightSpeed = Range.clip( rightSpeed, -1.0, +1.0 );

                driveSetPower( leftSpeed, rightSpeed );

                /* uncomment when you need the debug
                // Display drive status for the driver.
                control.telemetry.addData("Err/St","%5.1f/%5.1f",  error, steer);
                control.telemetry.addData("posit", "fr:%7d fl:%7d br:%7d bl:%7d ", f_right.getCurrentPosition(), f_left.getCurrentPosition(), b_right.getCurrentPosition(), b_left.getCurrentPosition());
                control.telemetry.addData("Speed","l:%5.2f r:%5.2f",  leftSpeed, rightSpeed);
                control.telemetry.addData("power","fr:%5.2f fl:%5.2f br:%5.2f bl:%5.2f", f_right.getPower(), f_left.getPower(), b_right.getPower(), b_left.getPower());
                control.telemetry.update();
                */
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            setDriveMode( DcMotor.RunMode.RUN_USING_ENCODER, 0 );
        }
    }

    public void gyroDriveArm ( double speed,
                            double distance,
                            double angle,
                            double TargetReached )
    {
        // Ensure that the opmode is still active
        if (control.opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            int clicks = driveInchesToClicks( distance );

            setDriveTargets( clicks, clicks, clicks, clicks );

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setDriveMode( DcMotor.RunMode.RUN_TO_POSITION, speed );

            // keep looping while we are still active, and BOTH motors are running.
            while (control.opModeIsActive() && driveIsBusy())
            {
                // adjust relative speed based on heading error.
                double error = angle - gyroGetAngle();
                double steer = error * P_DRIVE_COEFF;

                if (distance < 0)
                    steer *= -1.0;

                double leftSpeed  = speed - steer;
                double rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                leftSpeed  = Range.clip( leftSpeed,  -1.0, +1.0 );
                rightSpeed = Range.clip( rightSpeed, -1.0, +1.0 );

                driveSetPower( leftSpeed, rightSpeed );

                /* uncomment when you need the debug
                // Display drive status for the driver.
                control.telemetry.addData("Err/St","%5.1f/%5.1f",  error, steer);
                control.telemetry.addData("posit", "fr:%7d fl:%7d br:%7d bl:%7d ", f_right.getCurrentPosition(), f_left.getCurrentPosition(), b_right.getCurrentPosition(), b_left.getCurrentPosition());
                control.telemetry.addData("Speed","l:%5.2f r:%5.2f",  leftSpeed, rightSpeed);
                control.telemetry.addData("power","fr:%5.2f fl:%5.2f br:%5.2f bl:%5.2f", f_right.getPower(), f_left.getPower(), b_right.getPower(), b_left.getPower());
                control.telemetry.update();
                */
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            setDriveMode( DcMotor.RunMode.RUN_USING_ENCODER, 0 );

        }
    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void gyroResetAngle()
    {
        gyroLastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyroGlobalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double gyroGetAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - gyroLastAngles.firstAngle;

        if      (deltaAngle < -180)  deltaAngle += 360;
        else if (deltaAngle >  180)  deltaAngle -= 360;

        gyroGlobalAngle += deltaAngle;

        gyroLastAngles = angles;

        return gyroGlobalAngle;
    }

    public void gyroCalibrate()
    {
        BNO055IMU.Parameters gyroParams = new BNO055IMU.Parameters();

        gyroParams.mode                = BNO055IMU.SensorMode.IMU;
        gyroParams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParams.loggingEnabled      = false;

        gyro.initialize( gyroParams );

        // make sure the gyro is calibrated before continuing
        while (!control.isStopRequested() && gyro.isGyroCalibrated())
        {
            control.sleep(50);
            control.idle();
        }

        gyroResetAngle();
    }

    // arm code
}
