package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cvtest.GridVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

/* AutoBase
 * Basic structure of autonomous programs with computer vision and roadrunner
 */
@Autonomous(name = "AutoBase")
public class AutoBase extends LinearOpMode {

    /* * * * Computer Vision * * * */
    private VisionPortal visionPortal = null;
    private DetectionProcessor detectionProcessor = null;

    public void runOpMode() throws InterruptedException {

        /*
         * Computer vision will scan with the camera 1000 times to check for a block
         * It will then set the according position for the pixel deliveries
         */

        detectionProcessor = new DetectionProcessor(30, 30, DetectionProcessor.DetectionColor.BLUE, telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detectionProcessor);

        //scan 1000 times


        //close vision portal
        //visionPortal.close();

        // Wait for the DS start button to be touched.``
        waitForStart();
        //start roadrunner autonomous

        //test close
        visionPortal.close();
    }
}
