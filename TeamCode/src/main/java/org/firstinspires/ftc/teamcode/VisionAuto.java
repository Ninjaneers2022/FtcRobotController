package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous
public class VisionAuto extends LinearOpMode
{

    Ninjabot robot;
    OpenCvInternalCamera phoneCam;
    SleeveDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {

        phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        while(!opModeIsActive() && !isStopRequested()) {
        //press int
            telemetry.addData("AVG1", pipeline.Avg1());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
        //press play
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("AVG1", pipeline.Avg1());
            telemetry.update();

        // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

    }

    public static class SleeveDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the sleeve position
         */
        public enum SleevePos
        {
            ONE,
            TWO,
            THREE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);

        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);

        final int REGION_WIDTH = 80;
        final int REGION_HEIGHT = 40;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SleevePos position = SleevePos.ONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Imgproc.cvtColor(input,YCrCb, Imgproc.COLOR_BGR2HSV );
            Core.extractChannel(YCrCb, Cb, 2);


        }

        @Override
        public Mat processFrame(Mat input)
        {

            inputToCb(input);


            avg1 = (int) Core.mean(region1_Cb).val[0];


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            return input;
        }

        public int Avg1(){
            return avg1;
        }


        public SleevePos getAnalysis()
        {
            return position;
        }
    }
}