package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class Remote_Control extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Ninjabot robot;
        robot = new Ninjabot(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int i = 0;

        waitForStart();
        runtime.reset();

        ModernRoboticsI2cGyro gyro = null;

        // get starting position of  robot.liftMotor
        int Originalposlift = robot.liftMotor.getCurrentPosition();
        int ClicksToTop = 100;
        int ClicksToMed = 50;
        int ClicksTolow = 20;
        int ClicksToGround = 10;
        int ToptargetPos = Originalposlift + ClicksToTop ;
        int MedtargetPos = Originalposlift + ClicksToMed ;
        int LowtargetPos = Originalposlift + ClicksTolow ;
        int GroundtargetPos = Originalposlift + ClicksToGround ;
        double wristposition = 0.8;


        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // stall motors
        robot.liftMotor.setTargetPosition(LowtargetPos);
        robot.liftMotor.setPower(.5);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        while (opModeIsActive()) {
            double leftPower;
            double rightPower;

            //buttons and game pad on remote
            float yAxis = -gamepad1.left_stick_y;
            float xAxis = gamepad1.left_stick_x;
            boolean boost = gamepad1.dpad_up;
            boolean slow = gamepad1.dpad_down;
            boolean lift = gamepad1.a;
            boolean lower = gamepad1.y;
            boolean clawClose = gamepad1.b;
            boolean clawOpen = gamepad1.x;
            float wristOut = gamepad1.left_trigger;
            float wristIn = gamepad1.right_trigger;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            //gamepad2 input
            boolean armUp = gamepad2.y;
            boolean armDown = gamepad2.b;

            //double angle = yAxis/xAxis; double medSpeed = 0.2679; double lowSpeed = 0.0875;
            double maxSpeed = 0.4;//0.6,0.5
            double BOOST = 0;
            double minSpeed = 0;


            double upPower = Range.clip(gamepad1.left_trigger, -maxSpeed, maxSpeed);
            double downPower = Range.clip(gamepad1.right_trigger, -maxSpeed, maxSpeed);


            //If gamepad1.dpad_up button pressed, boost power
            if (boost == true) {
                BOOST = 1;
            } else {
                BOOST = 0;
            }

            //If gamepad1.dpad_down button pressed, cull power
            if (slow == true) {
                minSpeed = 0.3;
            } else {
                minSpeed = 0;
            }

            leftPower = Range.clip(yAxis - xAxis, -maxSpeed - BOOST + minSpeed, maxSpeed + BOOST - minSpeed);
            rightPower = Range.clip(yAxis + xAxis, -maxSpeed - BOOST + minSpeed, maxSpeed + BOOST - minSpeed);

            //joysticks
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            //buttons
            if (lift == false && lower == false) {
                robot.liftMotor.setPower(0);
                telemetry.addData("none pressed", robot.liftMotor.getCurrentPosition());
            }
            if (lift == true) {
                robot.liftMotor.setPower(1);
                telemetry.addData("y pressed", robot.liftMotor.getCurrentPosition());
            }
            if (lower == true) {
                robot.liftMotor.setPower(-1);
                telemetry.addData("a pressed", robot.liftMotor.getCurrentPosition());
                //lift set to position 20 for the lowest position
            }


            if (clawOpen == true) {

                double position = robot.claw.getPosition() - 0.05;
                robot.claw.setPosition(position);
                telemetry.addData("claw position open",position);
                telemetry.update();
            }
            if (clawClose == true) {
                double position = robot.claw.getPosition() + 0.05;
                robot.claw.setPosition(position);
                telemetry.addData("claw position close",position);
                telemetry.update();
            }

            wristposition = Range.clip(wristposition,0.4,0.8);
            if (wristOut > 0.3) {
                wristposition -= 0.1;
                robot.wrist.setDirection(Servo.Direction.REVERSE);
                // robot.wrist.setPosition(0.4);
            }
            if (wristIn > 0.3) {
                wristposition += 0.1 ;
                robot.wrist.setDirection(Servo.Direction.REVERSE);
                //robot.wrist.setPosition(0.8);
            }
            if (wristIn < 0.3 && wristOut < 0.3) {
                double display = robot.wrist.getPosition();
                // telemetry.addData("sevo", display);
               // telemetry.update();
            }

            if (up) {
                robot.gyroTurn(50, -90);
            }
            if (down) {
                robot.gyroTurn(50, 90);
            }
            if (left) {
                robot.gyroTurn(50, 0);
            }
            if (right) {
                robot.gyroTurn(50, 180);
            }
            if (armUp) {
                i += 1;
            }
            if (armDown) {
                i -= 1;
            }


        }


            sleep(10);
        }


    }