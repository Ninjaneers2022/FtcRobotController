/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

/*
 * This sample demonstrates a basic sleeve position
 */
//@TeleOp
@Autonomous
@Disabled
public class VisionAuto extends LinearOpMode {
    Ninjabot robot;
    OpenCvInternalCamera phoneCam;
    SleeveDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        int FORWARD = 1;
        int BACKWARD = 3;
        int ROTATE_LEFT = 5;
        int ROTATE_RIGHT = 6;
        int TANK_LEFT = 7;
        int TANK_RIGHT = 8;
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SleeveDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addData("AVG1", pipeline.Avg1());

            telemetry.update();

        }
        waitForStart();

        int sleeve = pipeline.Avg1();
        sleep(50);

        if (sleeve <= 150) {
            robot.driveTo(550, ROTATE_LEFT); //950 is equal to a 180 degree turn of the robot in rotate turns
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        }
        else {
            robot.driveTo(550, ROTATE_RIGHT); //950 is equal to a 180 degree turn of the robot in rotate turns
            while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
        }

    }


    public static class SleeveDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the sleeve position
         */
        public enum SleevePos {
            ONE,
            TWO,
            THREE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);


        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109, 98);

        static final int REGION_WIDTH = 80;
        static final int REGION_HEIGHT = 40;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
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
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Imgproc.cvtColor(input,YCrCb, Imgproc.COLOR_BGR2HSV );
            Core.extractChannel(YCrCb, Cb, 2);


        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        /*
        private Mat ycrcbMat       = new Mat();
        private Mat binaryMat      = new Mat();
        private Mat maskedInputMat = new Mat();

        public Scalar lower = new Scalar(94, 80, 2);
        public Scalar upper = new Scalar(126, 255, 255);

        public Scalar lgreen = new Scalar(25, 52, 72);
        public Scalar hgreen = new Scalar(102, 255, 255);
        */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);
            //original
            //Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

            //Core.inRange(ycrcbMat, lower, upper, binaryMat);

            //original
            //Core.inRange(ycrcbMat, lgreen, hgreen, binaryMat);


            //original
            //maskedInputMat.release();

            //original
            //Core.bitwise_and(input, input, maskedInputMat, binaryMat);



            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];


            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2);
            /*Imgproc.rectangle(
                    maskedInputMat, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
             */


            //return maskedInputMat;



            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }


        public int Avg1() {
            return avg1;
        }


        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SleevePos getAnalysis() {
            return position;
        }
    }
}
