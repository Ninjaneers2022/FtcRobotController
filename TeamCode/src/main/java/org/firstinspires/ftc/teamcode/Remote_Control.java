package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        waitForStart();
        runtime.reset();

        ModernRoboticsI2cGyro gyro = null;

        while (opModeIsActive()){
            double leftPower;
            double rightPower;

            //buttons and game pad on remote
            float yAxis = -gamepad1.left_stick_y;
            float xAxis = gamepad1.left_stick_x;
            boolean boost = gamepad1.a;
            boolean slow = gamepad1.y;
            boolean clawClose = gamepad1.b;
            boolean clawOpen = gamepad1.x;
            boolean optiArm = gamepad1.left_bumper;
            boolean stopArm = gamepad1.right_bumper;

            //double angle = yAxis/xAxis; double medSpeed = 0.2679; double lowSpeed = 0.0875;
            double maxSpeed = 0.6;
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

            int i = 0;
            //buttons
            //making the robot face the cardinal directions of the board when buttons pressed
            if (optiArm == true) {
                i += 1;
            }
            if (clawOpen == true){
            //    robot.claw.setPosition(0.8);
            }
            if (clawClose == true){
            //    robot.claw.setPosition(-0.2);//this claw position
            }


            sleep(10);
        }
    }
}
