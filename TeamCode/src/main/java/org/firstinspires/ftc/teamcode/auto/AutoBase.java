package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

/* AutoBase
 * Basic structure of autonomous programs with computer vision and roadrunner
 */
@Autonomous(name = "AutoBase")
public class AutoBase extends LinearOpMode {

    /* Attachments */
    private ArmControl Arm = null;
    private ClawControl Claw = null;

    /* * * * Computer Vision * * * */
    private VisionPortal visionPortal = null;
    private DetectionProcessor detectionProcessor = null;

    /* * * * Roadrunner * * * */
    private int spikeMark = 0; //1: left, 2: middle, 3: right

    public void runOpMode() throws InterruptedException {

        /* Build roadrunner trajectories */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        TrajectorySequence leftSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-16.5, 0, Math.toRadians(45)))
                .back(11.1)
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(0)))
                .splineTo(new Vector2d(-36, 30), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Arm.autoDeliver();
                    Claw.autoDeliver();
                })
                .build();
        TrajectorySequence leftSeq2 = drive.trajectorySequenceBuilder(leftSeq1.end())
                .strafeLeft(33.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.reset();
                    Claw.reset();
                })
                .build();
        TrajectorySequence leftSeq3 = drive.trajectorySequenceBuilder(leftSeq2.end())
                .back(16)
                .build();


        /*
         * Computer vision will scan with the camera 1000 times to check for a block
         * It will then set the according position for the pixel deliveries
         */

        detectionProcessor = new DetectionProcessor(30, 30, DetectionProcessor.DetectionColor.BLUE, telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detectionProcessor);

        //scan for spikemark
        spikeMark = detectionProcessor.scanForSpikeMark();

        //close vision portal
        //visionPortal.close();

        // Wait for the DS start button to be touched.``
        waitForStart();
        //close vision portal if not already done
        visionPortal.close();

        //init attachments
        Arm.init();
        Claw.init();

        //start roadrunner
        if(spikeMark == 1) {
            drive.followTrajectorySequence(leftSeq1);
            waitForArm();
            drive.followTrajectorySequence(leftSeq2);
            waitForArm();
            drive.followTrajectorySequence(leftSeq3);
        }
    }



    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && !Arm.finishedDelivery()) {
            //wait
        }
        sleep(1000);
    }
}
