package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


/* AutoBase
 * Basic structure of autonomous programs with computer vision and roadrunner
 */
@Autonomous(name = "AutoBase")
public class AutoBase extends LinearOpMode {

    /* * * * Computer Vision * * * */
    private VisionPortal visionPortal = null;
    private DetectionProcessor detectionProcessor = null;
    private boolean saturatedRegion = false;

    /* * * * Roadrunner * * * */
    private int spikeMark = 0; //1: left, 2: middle, 3: right

    public void runOpMode() throws InterruptedException {

        /*
         * Computer vision will scan with the camera 1000 times to check for a block
         * It will then set the according position for the pixel deliveries
         */

        detectionProcessor = new DetectionProcessor(30, 30, DetectionProcessor.DetectionColor.RED, DetectionProcessor.RelativePos.RIGHT, telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detectionProcessor);

        //wait for camera to be ready
        waitForProcessor();

        //scan region
        saturatedRegion = detectionProcessor.scanRegion(12, 13, 5, 6, 0.5);

        //scan for spikemark
        //spikeMark = detectionProcessor.scanForSpikeMark();

        // Wait for the DS start button to be touched.
        waitForStart();
        telemetry.addData("saturatedRegion", saturatedRegion);
        telemetry.update();

        //close vision portal if not already done
        visionPortal.close();

    }

    //wait for the camera processor to start working
    private void waitForProcessor() {
        while(opModeInInit() && !detectionProcessor.getActivity()) {
            telemetry.addLine("Waiting for camera...");
            telemetry.update();
        }
        telemetry.addLine("Camera opened.");
        telemetry.update();
    }
}
