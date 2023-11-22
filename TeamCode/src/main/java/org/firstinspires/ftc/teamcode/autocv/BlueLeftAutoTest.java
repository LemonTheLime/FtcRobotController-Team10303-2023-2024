package org.firstinspires.ftc.teamcode.autocv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//this program starts on the red right side and follows middle pixel protocol
@Autonomous(name="BlueLeftAutoTest")
public class BlueLeftAutoTest extends LinearOpMode {
    private ArmControl Arm = null;
    private ClawControl Claw = null;
    private int spikeMark = 3;

    //trajectory fields
    private SampleMecanumDrive drive;
    private Trajectory rTraj1, rTraj1prime, rTraj2, rTraj3, rTraj4, rTraj5, rTraj6 = null;
    private Trajectory mTraj1, mTraj2, mTraj3, mTraj4, mTraj5, mTraj6 = null;
    private Trajectory lTraj1, lTraj2, lTraj3, lTraj4, lTraj5, lTraj6 = null;


    public void runOpMode() throws InterruptedException {

        //attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        //drivetrain
        drive = new SampleMecanumDrive(hardwareMap);

        //trajectories are flipped direction

        if(spikeMark == 1) {
            //left pixel trajectories
            buildLeftPixelTraj();
        } else if(spikeMark == 2) {
            //middle pixel trajectories
            buildMiddlePixelTraj();
        } else if(spikeMark == 3) {
            //right pixel trajectories
            buildRightPixelTraj();
        }

        //start
        waitForStart();

        //init attachments
        Arm.init();
        Claw.init();
        //Claw.close();
        sleep(1000);

        if(spikeMark == 1) {
            followLeftPixelTraj();
        } else if(spikeMark == 2) {
            followMiddlePixelTraj();
        } else if(spikeMark == 3) {
            followRightPixelTraj();
        }
    }


    //build left pixel trajectories
    private void buildLeftPixelTraj() {
        lTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-26.5, 3, Math.toRadians(-25)))
                .build();
        lTraj2 = drive.trajectoryBuilder(lTraj1.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(-90)))
                .build();
        lTraj3 = drive.trajectoryBuilder(lTraj2.end())
                .back(29)
                .build();
        lTraj4 = drive.trajectoryBuilder(lTraj3.end())
                .strafeRight(7)
                .build();
        lTraj5 = drive.trajectoryBuilder(lTraj4.end())
                .strafeLeft(18.5)
                .build();
        lTraj6 = drive.trajectoryBuilder(lTraj5.end())
                .back(17)
                .build();
    }

    //follow left pixel trajectories
    private void followLeftPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(lTraj1);
        drive.followTrajectory(lTraj2);
        drive.followTrajectory(lTraj3);
        drive.followTrajectory(lTraj4);

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
        drive.followTrajectory(lTraj5);
        drive.followTrajectory(lTraj6);
    }

    //build middle pixel trajectories
    private void buildMiddlePixelTraj() {
        mTraj1 = drive.trajectoryBuilder(new Pose2d())
                .back(27.5)
                .build();
        mTraj2 = drive.trajectoryBuilder(mTraj1.end())
                .forward(10)
                .build();
        mTraj3 = drive.trajectoryBuilder(mTraj2.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .back(27.5)
                .build();
        mTraj4 = drive.trajectoryBuilder(mTraj3.end())
                .strafeLeft(11)
                .build();
        //after the delivery
        mTraj5 = drive.trajectoryBuilder(mTraj4.end())
                .strafeRight(26)
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
        drive.turn(Math.toRadians(90));
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

    //build right pixel trajectories
    private void buildRightPixelTraj() {
        rTraj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-16.5, 0, Math.toRadians(45)))
                .build();
        rTraj1prime = drive.trajectoryBuilder(rTraj1.end())
                .back(11.1)
                .build();
        rTraj2 = drive.trajectoryBuilder(rTraj1prime.end())
                .lineToSplineHeading(new Pose2d(-15, 0, Math.toRadians(-90)))
                .build();
        rTraj3 = drive.trajectoryBuilder(rTraj2.end())
                .back(29)
                .build();
        rTraj4 = drive.trajectoryBuilder(rTraj3.end())
                .strafeRight(22.5)
                .build();
        rTraj5 = drive.trajectoryBuilder(rTraj4.end())
                .strafeLeft(33.5)
                .build();
        rTraj6 = drive.trajectoryBuilder(rTraj5.end())
                .back(16)
                .build();
    }

    //follow right pixel trajectories
    private void followRightPixelTraj() {
        //deliver purple pixel
        drive.followTrajectory(rTraj1);
        drive.followTrajectory(rTraj1prime);
        drive.followTrajectory(rTraj2);
        drive.followTrajectory(rTraj3);
        drive.followTrajectory(rTraj4);

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
        drive.followTrajectory(rTraj5);
        drive.followTrajectory(rTraj6);
    }

    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && !Arm.finishedDelivery()) {
            //wait
        }
        sleep(1000);
    }

}
