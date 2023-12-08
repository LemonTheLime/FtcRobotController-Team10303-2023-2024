package org.firstinspires.ftc.teamcode.cvtest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name="ThresholdOpMode", group = "CV Test")
public class ThesholdOpMode extends LinearOpMode {

    private ThresholdProcessor thresholdProcessor = null;

    private VisionPortal visionPortal = null;

    public void runOpMode() throws InterruptedException {

        thresholdProcessor  = new ThresholdProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), thresholdProcessor);

        telemetry.addData(">", "Open camera stream.");
        telemetry.update();

        String lastKeyPressed = "none";

        //"a" button for red, "b" button for blue
        while(!opModeIsActive()) {
            if(gamepad1.a) {
                if(lastKeyPressed.equals("none")) {
                    lastKeyPressed = "a";
                    thresholdProcessor.setColorMode(ThresholdProcessor.ElementColor.RED);
                    telemetry.addLine("Red Threshold");
                    telemetry.update();
                }
            }
            if(gamepad1.b) {
                if(lastKeyPressed.equals("none")) {
                    lastKeyPressed = "b";
                    thresholdProcessor.setColorMode(ThresholdProcessor.ElementColor.BLUE);
                    telemetry.addLine("Blue Threshold");
                    telemetry.update();
                }
            }
            if(gamepad1.x) {
                if(lastKeyPressed.equals("none")) {
                    lastKeyPressed = "x";
                    thresholdProcessor.setColorMode(ThresholdProcessor.ElementColor.RED2);
                    telemetry.addLine("Red Threshold (other end of spectrum)");
                    telemetry.update();
                }
            }
            if(gamepad1.y) {
                if(lastKeyPressed.equals("none")) {
                    lastKeyPressed = "y";
                    thresholdProcessor.setColorMode(ThresholdProcessor.ElementColor.NONE);
                    telemetry.addLine("Combined Threshold");
                    telemetry.update();
                }
            }

            if(!(gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)) {
                lastKeyPressed = "none";
            }

        }

        // Wait for the DS start button to be touched.``
        waitForStart();

        while (opModeIsActive()) {
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }
}
