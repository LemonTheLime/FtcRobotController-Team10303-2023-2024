package org.firstinspires.ftc.teamcode.opmodes.AutonomousCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.auto.DetectionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

/* RedLeftPurpleOnly
 * Delivers purple pixel only
 */
@Autonomous(name = "RedLeft")
public class RedLeft extends LinearOpMode {

    /* * * * Attachments * * * */
    private ArmControl Arm = null;
    private ClawControl Claw = null;

    /* * * * Computer Vision * * * */
    private VisionPortal visionPortal = null;
    private DetectionProcessor detectionProcessor = null;

    /* * * * Roadrunner * * * */
    private int spikeMark = 0; //1: left, 2: middle, 3: right
    private SampleMecanumDrive drive;
    private Trajectory lTraj1, lTraj2, lTraj3, lTraj4, lTraj5, lTraj6, lTraj7 = null;
    private Trajectory mTraj1, mTraj2, mTraj3, mTraj4, mTraj5, mTraj6, mTraj7 = null;
    private Trajectory rTraj1, rTraj1prime, rTraj2, rTraj3, rTraj4, rTraj5, rTraj6, rTraj7 = null;

    public void runOpMode() throws InterruptedException {

        //create attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        // Build roadrunner trajectories
        drive = new SampleMecanumDrive(hardwareMap);
        buildLeftPixelTraj();
        buildMiddlePixelTraj();
        buildRightPixelTraj();

        //create vision portal and processor
        detectionProcessor = new DetectionProcessor(30, 30, DetectionProcessor.DetectionColor.RED, DetectionProcessor.RelativePos.LEFT, telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detectionProcessor);

        // Wait for the DS start button to be touched.
        waitForStart();

        //init attachments
        Arm.init();
        Claw.init();
        Claw.closeLeftClaw();
        Claw.closeRightClaw();
        sleep(500);

        //scan for spikemark if not given enough time in init
        scanSpikeMark();

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
        //visionPortal.close();
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

        //drive underneath truss
        lTraj3 = drive.trajectoryBuilder(lTraj2.end())
                .lineToLinearHeading(new Pose2d(-3, 0, Math.toRadians(-90)))
                .build();
        lTraj4 = drive.trajectoryBuilder(lTraj3.end())
                .lineTo(new Vector2d(-3, 25))
                .build();

        //drive close to backdrop
        lTraj5 = drive.trajectoryBuilder(lTraj4.end())
                .lineTo(new Vector2d(-3, 60))
                .build();
        lTraj6 = drive.trajectoryBuilder(lTraj5.end())
                .lineToConstantHeading(new Vector2d(-28, 81.5))
                .build();

        //move up to backdrop before delivering yellow pixel
        lTraj7 = drive.trajectoryBuilder(lTraj6.end())
                .back(3.5)
                .build();
    }

    //follow left pixel trajectories
    private void followLeftPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(lTraj1);
        drive.followTrajectory(lTraj2);

        //drive underneath truss
        drive.followTrajectory(lTraj3);
        drive.followTrajectory(lTraj4);

        //delay
        sleep(1000);

        //drive close to backdrop
        drive.followTrajectory(lTraj5);
        sleep(500);
        drive.followTrajectory(lTraj6);

        //extend arm
        Arm.autoDeliver2();
        Claw.autoDeliver2();
        waitForArm();
        sleep(1000);

        //move up to backdrop before delivering yellow pixel
        drive.followTrajectory(lTraj7);

        //deliver yellow pixel
        Claw.openLeftClaw();
        sleep(1000);

        //retract
        Arm.autoArmUp();
        waitForArm();
        Claw.reset();
        sleep(500);
        Arm.autoReset();
        waitForArm();
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

        //drive underneath truss
        mTraj3 = drive.trajectoryBuilder(mTraj2.end())
                .lineToLinearHeading(new Pose2d(-3, 0, Math.toRadians(-90)))
                .build();
        mTraj4 = drive.trajectoryBuilder(mTraj3.end())
                .lineTo(new Vector2d(-3, 25))
                .build();

        //drive close to backdrop
        mTraj5 = drive.trajectoryBuilder(mTraj4.end())
                .lineTo(new Vector2d(-3, 60))
                .build();
        mTraj6 = drive.trajectoryBuilder(mTraj5.end())
                .lineToConstantHeading(new Vector2d(-22, 81.5))
                .build();

        //move up to backdrop before delivering yellow pixel
        mTraj7 = drive.trajectoryBuilder(mTraj6.end())
                .back(3.5)
                .build();

    }

    //follow middle pixel trajectories
    private void followMiddlePixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(mTraj1);
        drive.followTrajectory(mTraj2);

        //drive underneath truss
        drive.followTrajectory(mTraj3);
        drive.followTrajectory(mTraj4);

        //delay
        sleep(1000);

        //drive close to backdrop
        drive.followTrajectory(mTraj5);
        sleep(500);
        drive.followTrajectory(mTraj6);

        //extend arm
        Arm.autoDeliver2();
        Claw.autoDeliver2();
        waitForArm();
        sleep(1000);

        //move up to backdrop before delivering yellow pixel
        drive.followTrajectory(mTraj7);

        //deliver yellow pixel
        Claw.openLeftClaw();
        sleep(1000);

        //retract
        Arm.autoArmUp();
        waitForArm();
        Claw.reset();
        sleep(500);
        Arm.autoReset();
        waitForArm();
    }

    //build right pixel trajectories
    private void buildRightPixelTraj() {
        //deliver purple pixel
        rTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-16.5, 0, Math.toRadians(-45)))
                .build();
        rTraj1prime = drive.trajectoryBuilder(rTraj1.end())
                .back(10.5)
                .build();
        rTraj2 = drive.trajectoryBuilder(rTraj1prime.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(0)))
                .build();

        //drive underneath truss
        rTraj3 = drive.trajectoryBuilder(rTraj2.end())
                .lineToLinearHeading(new Pose2d(-3, 0, Math.toRadians(-90)))
                .build();
        rTraj4 = drive.trajectoryBuilder(rTraj3.end())
                .lineTo(new Vector2d(-3, 25))
                .build();

        //drive close to backdrop
        rTraj5 = drive.trajectoryBuilder(rTraj4.end())
                .lineTo(new Vector2d(-3, 60))
                .build();
        rTraj6 = drive.trajectoryBuilder(rTraj5.end())
                .lineToLinearHeading(new Pose2d(-13, 81.5, Math.toRadians(-90)))
                .build();

        //move up to backdrop before delivering yellow pixel
        rTraj7 = drive.trajectoryBuilder(rTraj6.end())
                .back(3.5)
                .build();

    }

    //follow right pixel trajectories
    private void followRightPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(rTraj1);
        drive.followTrajectory(rTraj1prime);
        drive.followTrajectory(rTraj2);

        //drive underneath truss
        drive.followTrajectory(rTraj3);
        drive.followTrajectory(rTraj4);

        //delay
        sleep(1000);

        //drive close to backdrop
        drive.followTrajectory(rTraj5);
        sleep(500);
        drive.followTrajectory(rTraj6);

        //extend arm
        Arm.autoDeliver2();
        Claw.autoDeliver2();
        waitForArm();
        sleep(1000);

        //move up to backdrop before delivering yellow pixel
        drive.followTrajectory(rTraj7);

        //deliver yellow pixel
        Claw.openLeftClaw();
        sleep(1000);

        //retract
        Arm.autoArmUp();
        waitForArm();
        Claw.reset();
        sleep(500);
        Arm.autoReset();
        waitForArm();
    }
    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && Arm.autoIsBusy()) {
            //wait
        }
        sleep(10);
    }

    //wait for the camera processor to start working
    private void waitForProcessor() {
        while(opModeIsActive() && !detectionProcessor.getActivity()) {
            telemetry.addLine("Waiting for camera...");
            telemetry.update();
        }
        telemetry.addLine("Camera opened.");
        telemetry.update();
    }

    //scan the spikemark
    private void scanSpikeMark() {
        if(!detectionProcessor.getActivity()) {
            //wait for camera processor
            waitForProcessor();
        }

        //scan for spikemark 10 times with a 10 ms delay and print selection
        for(int i = 0; i < 10; i++) {
            spikeMark = detectionProcessor.scanForSpikeMark();
            sleep(10);
        }
        telemetry.addData("Spike mark selected", spikeMark);
        telemetry.update();
    }
}
