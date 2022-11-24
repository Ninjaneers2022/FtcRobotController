
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import android.graphics.drawable.GradientDrawable;
import com.qualcomm.robotcore.hardware.GyroSensor;

import com.qualcomm.robotcore.util.ElapsedTime;



public class Ninjabot
{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public Servo rightElbow          = null;
    public Servo leftElbow          = null;
    public Servo wrist          = null;
    public Servo claw          = null;
    public DcMotor liftMotor    = null;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    private static final int FORWARD = 1;
    private static final int BACKWARD = 3;
    private static final int ROTATE_LEFT = 5;
    private static final int ROTATE_RIGHT = 6;
    private static final int TANK_LEFT= 7;
    private static final int TANK_RIGHT= 8;

    public static final double WheelD = 3.533;
    public static final int square = convert(24);
    public static final int RIGHTANGLE = convert(24);


    static final int REV_ROBOTICS_HDHEX_MOTOR   = 28; // ticks per rotation
    static final int REV_ROBOTICS_HDHEX_20_to_1 = REV_ROBOTICS_HDHEX_MOTOR * 20;

    static final int DRIVE_MOTOR_TICK_COUNTS    = REV_ROBOTICS_HDHEX_20_to_1;
    static final double WHEEL_DIAMETER          = 3.6;

    static final int REV_ROBOTICS_COREHEX_MOTOR   = 4; // ticks per rotation
    static final int REV_ROBOTICS_COREHEX_72_to_1 = REV_ROBOTICS_COREHEX_MOTOR * 72;
    static final int LiftCounts                    = REV_ROBOTICS_COREHEX_72_to_1;

    static final Scalar BLUE = new Scalar(0, 0, 255);

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    LinearOpMode control        =  null;
    BNO055IMU gyro;

    //needed for color detection code
    OpenCvInternalCamera phoneCam;

    public Orientation gyroLastAngle = null;
    private ElapsedTime period  = new ElapsedTime();
    static final double     P_DRIVE_COEFF           = 0.0125;

    /* Constructor */
    public SleeveDetermine sleeveNumber = null;

    public Ninjabot(HardwareMap map, LinearOpMode ctrl ){
        init(map, ctrl);
    }

    // save the location of everything to the hardware map
    public void init(HardwareMap ahwMap, LinearOpMode ctrl )
    {
        // Save reference to Hardware map
        hwMap   = ahwMap;
        control = ctrl;
        sleeveNumber = new SleeveDetermine();

        leftDrive = hwMap.get(DcMotor.class, "RD");
        rightDrive = hwMap.get(DcMotor.class, "LD");
        claw = hwMap.get(Servo.class,"claw");
        wrist = hwMap.get(Servo.class, "wrist");
        rightElbow = hwMap.get(Servo.class, "Relbow");
        leftElbow = hwMap.get(Servo.class, "Lelbow");
        liftMotor = hwMap.get(DcMotor.class, "lift");
        //spinner = hwMap.get(DcMotor.class, "spinner");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hwMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);


        leftDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        gyroLastAngle = new Orientation();
        //gyroGlobalAngle = 0.0;
        sleeveNumber.initialize(hwMap, ctrl);

        int Originalposlift = liftMotor.getCurrentPosition();
    }

    public void resetAngle(){
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
    }

    public double getHeading(){
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getAngle() {

        Orientation orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle >180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        control.telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();
        double error = degrees;

        while(control.opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -0.3: 0.3);
            leftDrive.setPower(motorPower);
            rightDrive.setPower(-motorPower);
            control.telemetry.addData("error", error);
            control.telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }


    public void turnTo(double degrees){
        Orientation orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if(error > 180) {
            error -= 360;
        } else if (error <= -180){
            error += 360;
        }

        turn(error);
    }

    // as given

    public void gyroTurn(double speed, double angle){
        while (control.opModeIsActive() && !gyroOnHeading(speed, angle, 0.1)){
            control.telemetry.update();
        }
    }
    //as given
    boolean gyroOnHeading(double speed, double angle, double PCoeff)
    {
        double  error;
        double  steer;
        boolean onTarget = false;
        double  leftSpeed;
        double  rightSpeed;

        error = gyroGetError(angle);

        if (Math.abs(error) <= 1)
        {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = gyroGetSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        driveSetPower( leftSpeed, rightSpeed );

        control.telemetry.addData("Target", "%5.2f", angle);
        control.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        control.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    //as given
    public double gyroGetError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntergratedZValue();
        //while (robotError > 180)  robotError -= 360;
        //while (robotError <= -180) robotError += 360;
        Orientation angles =
                gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - gyroLastAngle.firstAngle;

        if      (deltaAngle < -180)  deltaAngle += 360;
        else if (deltaAngle >  180)  deltaAngle -= 360;

        robotError=0; // this needs to be deleted later
        return robotError;
    }
    //as given
    public double gyroGetSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }
    //as given
    public void driveSetPower( double leftPower, double rightPower )
    {
        rightDrive.setPower(rightPower);
        leftDrive.setPower(leftPower);
    }

    public static int convert(int inches) {
        // wheel circumferance / 360 = distance per degree        distanc / distance per degree
        double inchToDegrees = inches / (WheelD * 3.14159 / 360);
        return (int) inchToDegrees;
    }

    public static boolean inRange(double test, double median, double range){
        boolean answer = false;
        if (test > median - range){
            if (test < median + range){
                answer = true;
            }
        }
        else{
            answer = false;
        }
        return answer;
    }

    public void driveTo(int distance, int dir) {
        if (dir == BACKWARD) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        }     // to reversing
        else if (dir == FORWARD) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        }   // to going forward
        else if (dir == ROTATE_LEFT) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        }     // to rotate the left wheels
        else if (dir == ROTATE_RIGHT) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        }     // to rotate the right wheels
        else if (dir == TANK_LEFT) {
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        }
        else if (dir == TANK_RIGHT) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
        }
    }

    public boolean targetReached() {
        int average = Math.abs(leftDrive.getTargetPosition() - leftDrive.getCurrentPosition());
        average += Math.abs(rightDrive.getTargetPosition() - rightDrive.getCurrentPosition());
        average = average / 2;
        return (average < 50);
    }
    public void updateWheelTelemetry() {
        control.telemetry.addData("leftDrive", leftDrive.getTargetPosition() - leftDrive.getCurrentPosition());
        control.telemetry.addData("rightDrive", rightDrive.getTargetPosition() - rightDrive.getCurrentPosition());
        control.telemetry.update();
    }

    //color detection code here

    //gyro code here
    public double gyroTurn(int speed, double bearing){
        double startPosition = getHeading();
        double desiredPosition = bearing;
        //initial heading is zero
        int degreesToTurn = Math.toIntExact((long) ((desiredPosition - startPosition)*(1100/360)));

        return degreesToTurn;
    }
}