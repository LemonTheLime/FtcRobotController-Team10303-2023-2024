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

/* BlueLeft
 * Delivers purple and yellow pixels to corresponding spike marks and then parks.
 */
@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode {

    /* * * * Attachments * * * */
    private ArmControl Arm = null;
    private ClawControl Claw = null;

    /* * * * Computer Vision * * * */
    private VisionPortal visionPortal = null;
    private DetectionProcessor detectionProcessor = null;

    /* * * * Roadrunner * * * */
    private int spikeMark = 0; //1: left, 2: middle, 3: right
    private SampleMecanumDrive drive;
    private Trajectory lTraj1, lTraj2, lTraj3, lTraj4, lTraj5, lTraj6 = null;
    private Trajectory mTraj1, mTraj2, mTraj3, mTraj4, mTraj5 = null;
    private Trajectory rTraj1, rTraj1prime, rTraj2, rTraj3, rTraj4, rTraj5 = null;

    public void runOpMode() throws InterruptedException {

        //create attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        //Build roadrunner trajectories
        drive = new SampleMecanumDrive(hardwareMap);
        buildLeftPixelTraj();
        buildMiddlePixelTraj();
        buildRightPixelTraj();

        //create vision portal and processor
        detectionProcessor = new DetectionProcessor(30, 30, DetectionProcessor.DetectionColor.BLUE, DetectionProcessor.RelativePos.LEFT, telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), detectionProcessor);

        //scan for spikemark
        //scanSpikeMark();

        // Wait for the DS start button to be touched.
        waitForStart();

        //scan for spikemark if not finished in init
        scanSpikeMark();

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
                .lineToSplineHeading(new Pose2d(-20.5, -3, Math.toRadians(25)))
                .build();
        lTraj2 = drive.trajectoryBuilder(lTraj1.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(90)))
                .build();
        //deliver yellow pixel to backdrop
        //old -29
        lTraj3 = drive.trajectoryBuilder(lTraj2.end(), true)
                .splineTo(new Vector2d(-17.5, -30), Math.toRadians(-90))
                .addTemporalMarker(1, () -> {
                    Arm.autoDeliver();
                    Claw.autoDeliver();
                })
                .build();
        //close arm and park
        lTraj4 = drive.trajectoryBuilder(lTraj3.end())
                .strafeRight(17)
                .addTemporalMarker(0, () -> {
                    Arm.autoReset();
                })
                .build();
        lTraj5 = drive.trajectoryBuilder(lTraj4.end())
                .back(16)
                .build();
    }

    //follow left pixel trajectories
    private void followLeftPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(lTraj1);
        drive.followTrajectory(lTraj2);
        //deliver yellow pixel to backdrop
        drive.followTrajectory(lTraj3);
        //wait for arm to deliver and open claw
        waitForArm();
        Claw.openRightClaw();
        sleep(1000);
        //retract
        Arm.autoArmUp();
        Claw.reset();
        waitForArm();
        //park
        drive.followTrajectory(lTraj4);
        drive.followTrajectory(lTraj5);
        waitForArm();
    }

    //build middle pixel trajectories
    private void buildMiddlePixelTraj() {
        //deliver purple pixel
        mTraj1 = drive.trajectoryBuilder(new Pose2d())
                .back(27.5)
                .build();
        mTraj2 = drive.trajectoryBuilder(mTraj1.end())
                .forward(15)
                .build();
        //deliver yellow pixel to backdrop
        mTraj3 = drive.trajectoryBuilder(mTraj2.end(), true)
                .splineTo(new Vector2d(-24, -30.5), Math.toRadians(-90))
                .addTemporalMarker(1, () -> {
                    Arm.autoDeliver();
                    Claw.autoDeliver();
                })
                .build();
        //close arm and park
        mTraj4 = drive.trajectoryBuilder(mTraj3.end())
                .strafeRight(23)
                .addTemporalMarker(0, () -> {
                    Arm.autoReset();
                    Claw.reset();
                })
                .build();
        mTraj5 = drive.trajectoryBuilder(mTraj4.end())
                .back(16)
                .build();
    }

    //follow middle pixel trajectories
    private void followMiddlePixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(mTraj1);
        drive.followTrajectory(mTraj2);
        //deliver yellow pixel to backdrop
        drive.followTrajectory(mTraj3);
        //wait for arm to deliver and open claw
        waitForArm();
        Claw.openRightClaw();
        sleep(1000);
        //retract
        Arm.autoArmUp();
        waitForArm();
        //park
        drive.followTrajectory(mTraj4);
        drive.followTrajectory(mTraj5);
        waitForArm();
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
        //deliver yellow pixel to backdrop
        rTraj3 = drive.trajectoryBuilder(rTraj2.end(), true)
                .splineTo(new Vector2d(-30, -31), Math.toRadians(-90))
                .addTemporalMarker(1, () -> {
                    Arm.autoDeliver();
                    Claw.autoDeliver();
                })
                .build();
        //close arm and park
        rTraj4 = drive.trajectoryBuilder(rTraj3.end())
                .strafeRight(30)
                .addTemporalMarker(0, () -> {
                    Arm.autoReset();
                    Claw.reset();
                })
                .build();
        rTraj5 = drive.trajectoryBuilder(rTraj4.end())
                .back(16)
                .build();
    }

    //follow right pixel trajectories
    private void followRightPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(rTraj1);
        drive.followTrajectory(rTraj1prime);
        drive.followTrajectory(rTraj2);
        //deliver yellow pixel to backdrop
        drive.followTrajectory(rTraj3);
        //wait for arm to deliver and open claw
        waitForArm();
        Claw.openRightClaw();
        sleep(1000);
        //retract
        Arm.autoArmUp();
        waitForArm();
        //parkract and park
        drive.followTrajectory(rTraj4);
        drive.followTrajectory(rTraj5);
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
