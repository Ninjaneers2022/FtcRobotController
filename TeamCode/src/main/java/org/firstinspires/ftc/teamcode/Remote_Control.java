package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

        robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int i = 0;

        waitForStart();
        runtime.reset();

        ModernRoboticsI2cGyro gyro = null;

        while (opModeIsActive()){
            double leftPower;
            double rightPower;

            //buttons and game pad on remote
            float yAxis = -gamepad1.left_stick_y;
            float xAxis = gamepad1.left_stick_x;
            boolean boost = gamepad1.dpad_up;
            boolean slow = gamepad1.dpad_down;
            boolean lift = gamepad1.y;
            boolean lower = gamepad1.a;
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
            double maxSpeed = 0.5;//0.6
            double BOOST = 0;
            double minSpeed = 0;
            boolean spinPower = false;
            double speedspin = 0;
            spinPower = gamepad1.dpad_left;
            if(spinPower == true){
                speedspin = 1;
            }
            else if(gamepad1.dpad_right == true){
                speedspin = -1;
            }
            else{
                speedspin = 0;
            }
            //robot.spinner.setPower(speedspin);


            //double arm_angle = 0;
          // if(gamepad1.left_trigger == true){
          //      arm_angle = 0.6;
          //  }
         //   else if(gamepad1.right_trigger == true){
         //       arm_angle = -0.6;
         //   }
         //   else{
        //        arm_angle = 0;
         //   }

         //   robot.liftArm.setPower(arm_angle);




            //determining the power based on degree on angle on joystick
            //if (medSpeed <= angle & angle <= -medSpeed){
            //    maxSpeed = 0.8;
            //}
            //else if (lowSpeed <= angle & angle <= -lowSpeed){
            //    maxSpeed = 0.6;
            //}
            //else{
            //    maxSpeed = 0.4;
            //}///////////////////////////////////////
            double upPower = Range.clip( gamepad1.left_trigger , -maxSpeed, maxSpeed);
            double downPower = Range.clip( gamepad1.right_trigger, -maxSpeed, maxSpeed);
            if(upPower > 0){

            //    robot.liftArm.setPower(upPower);
            } else if(downPower > 0){
            //    robot.liftArm.setPower(-downPower);
            }else{
            //    robot.liftArm.setPower(0);
            }
            //robot.liftArm.setPower();///////////////////////


            //If A button pressed, boost power
            if (boost == true) {
                BOOST = 1;
            } else {
                BOOST = 0;
            }

            //If Y button pressed, cull power
            if (slow == true) {
                minSpeed = 0.3;
            } else {
                minSpeed = 0;
            }

            leftPower   = Range.clip(yAxis + xAxis, -maxSpeed-BOOST+minSpeed, maxSpeed+BOOST-minSpeed);
            rightPower  = Range.clip(yAxis - xAxis, -maxSpeed-BOOST+minSpeed, maxSpeed+BOOST-minSpeed);

            //joysticks
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            //buttons
            if (lift == true){
                robot.liftMotor.setPower(0.7);
            }
            if (lower == true){
                robot.liftMotor.setPower(-0.8);
            }

            if (lift == false && lower == false){
                robot.liftMotor.setPower(0);
            }
            if (clawOpen == true){
                robot.claw.setPosition(30);//40, 25, 30
            }
            if (clawClose == true){
                robot.claw.setPosition(40);
            }
            if (wristOut > 0.3){
                robot.wrist.setDirection(Servo.Direction.REVERSE);
                robot.wrist.setPosition(0.3);
            }
            if (wristIn > 0.3){
                robot.wrist.setDirection(Servo.Direction.REVERSE);
                robot.wrist.setPosition(0.9);
            }
            if (wristIn < 0.3 && wristOut < 0.3){
                double display = robot.wrist.getPosition();
                telemetry.addData("sevo", display);
                telemetry.update();
            }
            if (up){
                robot.gyroTurn(50, -90);
            }
            if (down){
                robot.gyroTurn(50, 90);
            }
            if (left){
                robot.gyroTurn(50, 0);
            }
            if (right){
                robot.gyroTurn(50, 180);
            }
            if (armUp) {
                i += 1;
            }
            if (armDown) {
                i -= 1;
            }
            int multiplyer = 30;
            Range.clip(i, 0, 3);
            while (robot.liftMotor.getCurrentPosition() != i * multiplyer) {
                double difference = robot.liftMotor.getCurrentPosition() - i * multiplyer;
                robot.liftMotor.setPower(difference);
            }


            //making the robot face the cardinal directions of the board when buttons pressed
            //if (optiArm == true) {
            //    i += 1;
            //}
            //if (clawOpen == true){
            //    robot.claw.setPosition(0.8);
            //}
            //if (clawClose == true){
            //    robot.claw.setPosition(-0.2);//this claw position
            }

            sleep(10);
        }


    }