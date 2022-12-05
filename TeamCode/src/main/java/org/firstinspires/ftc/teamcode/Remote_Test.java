package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Remote Test", group = "Linear Opmode")
public class Remote_Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Ninjabot robot;
        robot = new Ninjabot(hardwareMap, this);

        double clawPosition = 0;
        double wristposition = 0;

        robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        runtime.reset();

        ModernRoboticsI2cGyro gyro = null;


        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // stall motors
        robot.liftMotor.setPower(1);
        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //buttons and game pad on remote
        boolean clawClose = gamepad1.b;
        boolean clawOpen = gamepad1.x;
        boolean wristOut = gamepad1.y;
        boolean wristIn = gamepad1.a;

        while (opModeIsActive()) {

            clawPosition = Range.clip(robot.claw.getPosition(), 0, 0.5);
            wristposition = Range.clip(robot.wrist.getPosition(),0,1);

            // telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Wrist     ", robot.wrist.getPosition());
            telemetry.addData("Math Wrist", wristposition);
            telemetry.addData("Claw     ", robot.claw.getPosition());
            telemetry.addData("Math Claw", clawPosition);
            telemetry.update();

            //claw
            if (gamepad2.x) {
                clawPosition -= 0.05;
                robot.claw.setPosition(clawPosition);
                sleep(200);
            }
            if (gamepad2.b) {
                clawPosition += 0.05;
                robot.claw.setPosition(clawPosition);
                sleep(200);
            }

            //wrist is attached to elbow
            if (gamepad2.a) {
                wristposition += 0.1;
                robot.wrist.setPosition(wristposition);
                sleep(200);
            }
            else if (gamepad2.y) {
                wristposition -= 0.1;
                robot.wrist.setPosition(wristposition);
                sleep(200);
            }
        }
    }
}