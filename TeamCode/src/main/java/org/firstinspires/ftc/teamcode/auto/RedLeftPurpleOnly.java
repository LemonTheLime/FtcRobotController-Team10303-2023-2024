package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

/* AutoBase
 * Basic structure of autonomous programs with computer vision and roadrunner
 */
@Autonomous(name = "RedLeftPurpleOnly")
public class RedLeftPurpleOnly extends LinearOpMode {

    /* Attachments */
    private ArmControl Arm = null;
    private ClawControl Claw = null;

    /* * * * Computer Vision * * * */
    private VisionPortal visionPortal = null;
    private DetectionProcessor detectionProcessor = null;

    /* * * * Roadrunner * * * */
    private int spikeMark = 0; //1: left, 2: middle, 3: right
    private SampleMecanumDrive drive;
    private Trajectory lTraj1, lTraj2 = null;
    private Trajectory mTraj1, mTraj2 = null;
    private Trajectory rTraj1, rTraj1prime, rTraj2 = null;

    public void runOpMode() throws InterruptedException {

        //attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        /* Build roadrunner trajectories */
        drive = new SampleMecanumDrive(hardwareMap);
        buildLeftPixelTraj();
        buildMiddlePixelTraj();
        buildRightPixelTraj();


        /*
         * Computer vision will scan with the camera 1000 times to check for a block
         * It will then set the according position for the pixel deliveries
         */

        detectionProcessor = new DetectionProcessor(30, 30, DetectionProcessor.DetectionColor.RED, telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detectionProcessor);

        //scan for spikemark
        spikeMark = 3;


        // Wait for the DS start button to be touched.``
        waitForStart();

        //init attachments
        Arm.init();
        Claw.init();
        Claw.closeLeftClaw();
        Claw.closeRightClaw();

        //start roadrunner
        if(spikeMark == 1) {
            //follow left trajectory
            followLeftPixelTraj();
        } else if(spikeMark == 2) {
            //follow middle trajectory
            followMiddlePixelTraj();
        } else if(spikeMark == 3) {
            //follow right sequence
            followRightPixelTraj();
        }

        //close camera
        visionPortal.close();
    }



    //build left pixel trajectories
    private void buildLeftPixelTraj() {
        //deliver purple pixel
        lTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-26.5, -3, Math.toRadians(25)))
                .build();
        lTraj2 = drive.trajectoryBuilder(lTraj1.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(0)))
                .build();
    }

    //follow left pixel trajectories
    private void followLeftPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(lTraj1);
        drive.followTrajectory(lTraj2);
    }

    //build middle pixel trajectories
    private void buildMiddlePixelTraj() {
        //deliver purple pixel
        mTraj1 = drive.trajectoryBuilder(new Pose2d())
                .back(27.5)
                .build();
        mTraj2 = drive.trajectoryBuilder(mTraj1.end())
                .forward(10)
                .build();
    }

    //follow middle pixel trajectories
    private void followMiddlePixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(mTraj1);
        drive.followTrajectory(mTraj2);
    }

    //build right pixel trajectories
    private void buildRightPixelTraj() {
        //deliver purple pixel
        rTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-16.5, 0, Math.toRadians(-45)))
                .build();
        rTraj1prime = drive.trajectoryBuilder(rTraj1.end())
                .back(11.1)
                .build();
        rTraj2 = drive.trajectoryBuilder(rTraj1prime.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(0)))
                .build();
    }

    //follow right pixel trajectories
    private void followRightPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(rTraj1);
        drive.followTrajectory(rTraj1prime);
        drive.followTrajectory(rTraj2);
    }
    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && !Arm.finishedDelivery()) {
            //wait
        }
        sleep(1000);
    }
}