package org.firstinspires.ftc.teamcode.cvtest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="GrayCV")
public class GrayCV extends LinearOpMode {

    private GrayProcessor grayProcessor = null;

    private VisionPortal visionPortal = null;

    public void runOpMode() throws InterruptedException {

        grayProcessor = new GrayProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), grayProcessor);

        telemetry.addData(">", "Open camera stream for gray stream.");
        telemetry.update();

        // Wait for the DS start button to be touched.``
        waitForStart();

        if (opModeIsActive()) {
            // ...
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }
}
