package org.firstinspires.ftc.teamcode.test.cvtest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name = "LearnVisionOpMode", group = "CV Test")
public class LearnVisionOpMode extends LinearOpMode {
    private LearnVisionProcessor learnVisionProcessor = null;

    private VisionPortal visionPortal = null;

    public void runOpMode() throws InterruptedException {

        learnVisionProcessor = new LearnVisionProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), learnVisionProcessor);

        telemetry.addData(">", "Open camera stream.");
        telemetry.update();

        while (!opModeIsActive()) {
            telemetry.addData("identified", learnVisionProcessor.getSelection());
            telemetry.addData("Left Avg", learnVisionProcessor.getAvgReading("left"));
            telemetry.addData("Middle Avg", learnVisionProcessor.getAvgReading("middle"));
            telemetry.addData("Right Avg", learnVisionProcessor.getAvgReading("right"));

            telemetry.update();
        }

        // Wait for the DS start button to be touched.``
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("identified", learnVisionProcessor.getSelection());
            telemetry.addData("Left Avg", learnVisionProcessor.getAvgReading("left"));
            telemetry.addData("Middle Avg", learnVisionProcessor.getAvgReading("middle"));
            telemetry.addData("Right Avg", learnVisionProcessor.getAvgReading("right"));

            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }
}
