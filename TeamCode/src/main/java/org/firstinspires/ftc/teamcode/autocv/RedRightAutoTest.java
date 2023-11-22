package org.firstinspires.ftc.teamcode.autocv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//this program starts on the red right side and follows middle pixel protocol
@Autonomous(name="RedRightAutoTest")
public class RedRightAutoTest extends LinearOpMode {
    private ArmControl Arm = null;
    private ClawControl Claw = null;
    private int spikeMark = 1;

    //trajectory fields
    private SampleMecanumDrive drive;
    private Trajectory rTraj1, rTraj2, rTraj3, rTraj4, rTraj5, rTraj6 = null;
    private Trajectory mTraj1, mTraj2, mTraj3, mTraj4, mTraj5, mTraj6 = null;
    private Trajectory lTraj1, lTraj1prime, lTraj2, lTraj3, lTraj4, lTraj5, lTraj6 = null;


    public void runOpMode() throws InterruptedException {

        //attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        //drivetrain
        drive = new SampleMecanumDrive(hardwareMap);

        //trajectories are flipped direction

        if(spikeMark == 1) {
            //right pixel trajectories
            buildRightPixelTraj();
        } else if(spikeMark == 2) {
            //middle pixel trajectories
            buildMiddlePixelTraj();
        } else if(spikeMark == 3) {
            //left pixel trajectories
            buildLeftPixelTraj();
        }

        //start
        waitForStart();

        //init attachments
        Arm.init();
        Claw.init();
        //Claw.close();
        sleep(1000);

        if(spikeMark == 1) {
            followRightPixelTraj();
        } else if(spikeMark == 2) {
            followMiddlePixelTraj();
        } else if(spikeMark == 3) {
            followLeftPixelTraj();
        }
    }


    //build right pixel trajectories
    private void buildRightPixelTraj() {
        //place purple pixel
        rTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-26.5, 3, Math.toRadians(-25)))
                .build();
        //move back
        rTraj2 = drive.trajectoryBuilder(rTraj1.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(-90)))
                .build();
        //move to backdrop and start the arm
        rTraj3 = drive.trajectoryBuilder(rTraj2.end(), true)
                .splineTo(new Vector2d(-22, 30), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    Arm.autoDeliver();
                    Claw.autoDeliver();
                })
                .build();
        //retract and park
        rTraj4 = drive.trajectoryBuilder(rTraj3.end())
                .strafeLeft(18.5)
                .addTemporalMarker(0, () -> {
                    Arm.reset();
                    Claw.reset();
                })
                .build();
        rTraj5 = drive.trajectoryBuilder(rTraj4.end())
                .back(17)
                .build();


    }

    //follow right pixel trajectories
    private void followRightPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(rTraj1);
        drive.followTrajectory(rTraj2);
        //move to backdrop
        drive.followTrajectory(rTraj3);

        waitForArm();
        //open the claw
        //Claw.open();
        sleep(1000);

        //retract and park
        drive.followTrajectory(rTraj4);
        drive.followTrajectory(rTraj5);
        waitForArm();
    }

    //build middle pixel trajectories
    private void buildMiddlePixelTraj() {
        mTraj1 = drive.trajectoryBuilder(new Pose2d())
                .back(27.5)
                .build();
        mTraj2 = drive.trajectoryBuilder(mTraj1.end())
                .forward(10)
                .build();
        mTraj3 = drive.trajectoryBuilder(mTraj2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .back(27.5)
                .build();
        mTraj4 = drive.trajectoryBuilder(mTraj3.end())
                .strafeRight(11)
                .build();
        //after the delivery
        mTraj5 = drive.trajectoryBuilder(mTraj4.end())
                .strafeLeft(26)
                .build();
        mTraj6 = drive.trajectoryBuilder(mTraj5.end())
                .back(18)
                .build();
    }

    //follow middle pixel trajectories
    private void followMiddlePixelTraj() {
        //follow the trajectories
        drive.followTrajectory(mTraj1);
        drive.followTrajectory(mTraj2);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(mTraj3);
        drive.followTrajectory(mTraj4);

        //deliver the pixel
        Arm.autoDeliver();
        Claw.autoDeliver();
        waitForArm();

        //open the arm
        //Claw.open();
        sleep(1000);

        //retract
        Arm.reset();
        Claw.reset();
        waitForArm();

        //park
        drive.followTrajectory(mTraj5);
        drive.followTrajectory(mTraj6);
    }

    //build left pixel trajectories
    private void buildLeftPixelTraj() {
        //position to push purple pixel to the left
        lTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-16.5, 0, Math.toRadians(45)))
                .build();
        lTraj1prime = drive.trajectoryBuilder(lTraj1.end())
                .back(11.1)
                .build();
        //spline back
        lTraj2 = drive.trajectoryBuilder(lTraj1prime.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(0)))
                .build();
        //spline to the backdrop, activate arm 1 second in
        lTraj3 = drive.trajectoryBuilder(lTraj2.end(), true)
                .splineTo(new Vector2d(-36, 30), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    Arm.autoDeliver();
                    Claw.autoDeliver();
                })
                .build();
        //start to close arm and begin to park
        lTraj4 = drive.trajectoryBuilder(lTraj3.end())
                .strafeLeft(33.5)
                .addTemporalMarker(0, () -> {
                    Arm.reset();
                    Claw.reset();
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
        drive.followTrajectory(lTraj1prime);
        drive.followTrajectory(lTraj2);

        //start arm while moving to backdrop
        drive.followTrajectory(lTraj3);

        //wait for arm to deliver and open claw
        waitForArm();
        //Claw.open();
        sleep(1000);

        //retract and park
        drive.followTrajectory(lTraj4);
        drive.followTrajectory(lTraj5);
        waitForArm();
    }

    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && !Arm.finishedDelivery()) {
            //wait
        }
        sleep(1000);
    }

}
