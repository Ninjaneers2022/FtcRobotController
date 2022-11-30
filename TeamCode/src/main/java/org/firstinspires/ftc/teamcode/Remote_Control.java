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


        waitForStart();
        runtime.reset();

        ModernRoboticsI2cGyro gyro = null;


        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // stall motors
        robot.liftMotor.setPower(1);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        double REPos = robot.rightElbow.getPosition();
        double LEPos = robot.leftElbow.getPosition();
        double clawPosition = robot.claw.getPosition();
        double wristposition = robot.wrist.getPosition();
        int liftPosition = robot.liftMotor.getCurrentPosition();


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
            float clawClose = gamepad1.right_trigger;
            float clawOpen = gamepad1.left_trigger;
            boolean wristOut = gamepad1.right_bumper;
            boolean wristIn = gamepad1.left_bumper;
            boolean elbowUp = gamepad1.x;
            boolean elbowDown = gamepad1.b;


            //gamepad2 input
            //boolean liftPos1 = gamepad2.dpad_down;
            //boolean liftPos2 = gamepad2.dpad_up;
            //boolean armUp = gamepad2.y;
            //boolean armDown = gamepad2.b;

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

            clawPosition = Range.clip(robot.claw.getPosition(), 0, 0.25);
            liftPosition = Range.clip(robot.liftMotor.getCurrentPosition(), -11000, 0);
            wristposition = Range.clip(wristposition,0,1);
            REPos = Range.clip(robot.rightElbow.getPosition(), 0.25, 1);
            LEPos = Range.clip(robot.leftElbow.getPosition(), 0, 0.75);

            // telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Lift", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Elbow R", robot.rightElbow.getPosition());
            telemetry.addData("Elbow L", robot.leftElbow.getPosition());
            telemetry.addData("Wrist", wristposition);
            telemetry.addData("Claw", clawPosition);
            telemetry .update();

            robot.claw.setPosition(clawPosition);
            robot.wrist.setPosition(wristposition);
            robot.liftMotor.setTargetPosition(liftPosition);
            //elbow
            robot.rightElbow.setPosition(REPos);
            robot.leftElbow.setPosition(LEPos);
            //joysticks
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            //buttons
            //lift
            if (lift == false && lower == false) {
                robot.liftMotor.setPower(0);
            }
            if (lift == true) {
                liftPosition += 100;
                robot.liftMotor.setPower(1);
            }
            if (lower == true) {
                liftPosition -= 100;
                robot.liftMotor.setPower(1);
            }

            //claw
            if (clawOpen > 0.8) {
                clawPosition -= 0.05;
            }
            if (clawClose > 0.8) {
                clawPosition += 0.05;
            }

            //wrist is attached to elbow
            if (wristOut) {
                wristposition += 0.005;
            }
            if (wristIn) {
                wristposition -= 0.005;
            }

            //elbow
            if (elbowUp == true) {
                LEPos -= 0.025;
                REPos += 0.025;
            }
            if (elbowDown == true) {
                LEPos += 0.025;
                REPos -= 0.05;
            }
        }
        }
    }