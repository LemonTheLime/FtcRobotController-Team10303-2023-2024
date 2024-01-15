package org.firstinspires.ftc.teamcode.test.cvtest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name = "GridVisionOpMode", group = "CV Test")
public class GridVisionOpMode extends LinearOpMode {

    private GridVisionProcessor gridVisionProcessor = null;

    private VisionPortal visionPortal = null;

    public void runOpMode() throws InterruptedException {

        gridVisionProcessor = new GridVisionProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), gridVisionProcessor);

        telemetry.addData(">", "Open camera stream for grid stream.");
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
