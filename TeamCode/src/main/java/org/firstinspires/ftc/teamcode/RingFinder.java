/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


/*
 * this code derived from the example code: TensorFlowObjectDetection.java
 */
public class RingFinder

{
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public float connum ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYHOTAL/////AAABmfAypoaQF0ZDom3mUJUh2DFLkYhESYANSN3lMNrX6D5lT5a/60emW/IT3/ICDP3ycjlCcfdk0F3V3DACH30qYbYggQuUWDAVKcRGffha3LHiBcmUfI2YWiMozzZ2NxRxkJZ1MC+GDuG6uipK1FhplzI9PpKem12RCwTLYzqvPuoB+UwYoaSv8gMq/Q1AwuN2dtWNd0FQhlHOO6nnSvDhBm3n9r6XQRE6LxUdSD/LJBsXz5h0fW48tQ0bm+hQlzrkrFvYxcJ/YVef/qWzGUrJvJ8hzZbsaqokFXoZEUsQMQtSrLuWODs73WMRK9w09nYe+NQ9FRHzKsQ3k4Z+Stn3WT6ynPNzmp4+WU7dIq5dTVnG";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    LinearOpMode control =  null;

    public boolean initialize(HardwareMap hwMap, LinearOpMode ctrl, String cameraName ) // "Webcam 1" for example
   // public boolean initialize // "Webcam 1" for example
    {
        control = ctrl;
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hwMap.get( WebcamName.class, cameraName );

        //  Instantiate the Vuforia engine
        if(( vuforia = ClassFactory.getInstance().createVuforia(parameters)) != null )
        {   //  Initialize the TensorFlow Object Detection engine
            int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;

            if ((tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)) != null) {   // load the models for the things to look for from the trained data.
                // the order of elements is important Quad must be first and Single must be second to
                // match the stored models.
                tfod.loadModelFromAsset(TFOD_MODEL_ASSET, "Quad", "Single");

                tfod.activate() ;

                //tfod.setZoom(1.5, 1.78);

                return (true);
            }
        }

        return( false );
    }

    public void magnify(double zoomfactor)
    {
        if (tfod != null)
        {
            tfod.setZoom(zoomfactor, 1.78);
        }
    }


    public void release()
    {
        if (tfod != null)
        {
            tfod.shutdown();
            tfod = null;
        }
    }

    // get count of rings currently in sight
    public int count()
    {
        connum = 0.0f;
        if (tfod != null)
        {   // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {   // step through the list of recognitions and display boundary info.
                float  maxConfidence = 0.0f;
                String maxLabel      = "none";

                for( Recognition recognition : updatedRecognitions )
                {
                    control.telemetry.addData( "Entry", "Name:%s Confidence:%1.2f", recognition.getLabel(), recognition.getConfidence());

                    if( maxConfidence < recognition.getConfidence())
                    {
                        maxConfidence = recognition.getConfidence();
                        maxLabel      = recognition.getLabel();
                    }
                }

                control.telemetry.addData( "Max", "Name:%s Confidence:%1.2f", maxLabel, maxConfidence );
                //control.telemetry.update();

                connum = maxConfidence;

                switch( maxLabel )
                {
                    case "Quad"   : return( 4 );
                    case "Single" : return( 1 );
                    default       : return( 0 );
                }
            }
            return( 0 ); // nothing found
        }
        return( -1 ); // not initialized
    }


}
