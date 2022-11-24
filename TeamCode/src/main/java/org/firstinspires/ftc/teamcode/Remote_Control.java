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


        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // stall motors
        robot.liftMotor.setTargetPosition(LowtargetPos);
        robot.liftMotor.setPower(1);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        double wristposition = robot.wrist.getPosition();


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
            float wristOut = gamepad1.right_trigger;
            float wristIn = gamepad1.left_trigger;
            //boolean up = gamepad1.dpad_up;
            //boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean elbowUp = gamepad2.y;
            boolean elbowDown = gamepad2.a;


            //gamepad2 input
            boolean liftPos1 = gamepad2.dpad_down;
            boolean liftPos2 = gamepad2.dpad_up;
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

            //joysticks
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            double clawPosition = Range.clip(robot.claw.getPosition(), 0, 0.45); //previously, 0.4 and 0.8
            int liftPosition = Range.clip(robot.liftMotor.getCurrentPosition(), -11000, 0);
            wristposition = Range.clip(wristposition,0.4,0.8);
            double REPos = Range.clip(robot.rightElbow.getPosition(), 0, 1);
            double LEPos = Range.clip(robot.leftElbow.getPosition(), 0, 1);

            // telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Lift", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Elbow", robot.rightElbow.getPosition());
            telemetry.addData("Wrist", wristposition);
            telemetry.addData("Claw", clawPosition);
            telemetry .update();

            //buttons

            //lift
            if (lift == false && lower == false) {
                robot.liftMotor.setPower(0);
            }
            if (lift == true) {
                liftPosition += 100;
                robot.liftMotor.setPower(1);
                robot.liftMotor.setTargetPosition(liftPosition);
            }
            if (lower == true) {

                liftPosition -= 100;
                robot.liftMotor.setPower(1);
                robot.liftMotor.setTargetPosition(liftPosition);
                //lift set to position 20 for the lowest position
            }

            //claw
            if (clawOpen == true) {
                clawPosition = robot.claw.getPosition() - 0.05;
                robot.claw.setPosition(clawPosition);
                sleep(200);
            }
            if (clawClose == true) {
                clawPosition = robot.claw.getPosition() + 0.05;
                robot.claw.setPosition(clawPosition);
                sleep(200);
            }

            //wrist
            if (wristOut > 0.3) {
                wristposition = robot.wrist.getPosition() - 0.05;
                robot.wrist.setPosition(wristposition);
                sleep(200);
                wristposition = Range.clip(wristposition,0.4,0.8);
            }
            if (wristIn > 0.3) {
                wristposition = robot.wrist.getPosition() + 0.05;
                robot.wrist.setPosition(wristposition);
                sleep(200);
                wristposition = Range.clip(wristposition,0.4,0.8);
            }

            //elbow
            if (elbowUp == true) {
                LEPos = robot.leftElbow.getPosition() + 0.05;
                REPos = robot.rightElbow.getPosition() +0.05;
                robot.rightElbow.setPosition(REPos);
                robot.leftElbow.setPosition(LEPos);
                sleep(200);
            }
            if (elbowDown == true) {
                LEPos = robot.leftElbow.getPosition() - 0.05;
                REPos = robot.rightElbow.getPosition() - 0.05;
                robot.rightElbow.setPosition(REPos);
                robot.leftElbow.setPosition(LEPos);
                sleep(200);
            }


        }


            sleep(10);
        }


    }