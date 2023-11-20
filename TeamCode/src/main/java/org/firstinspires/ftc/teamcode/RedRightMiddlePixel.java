package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//this program starts on the red right side and follows middle pixel protocol
@Autonomous(name="RedRightMiddlePixel")
public class RedRightMiddlePixel extends LinearOpMode {
    private ArmControl Arm = null;
    private ClawControl Claw = null;
    private int spikeMark = 2;

    //trajectory fields
    private SampleMecanumDrive drive;
    private Trajectory mTraj1, mTraj2, mTraj3, mTraj4, mTraj5, mTraj6 = null;

    public void runOpMode() throws InterruptedException {

        //attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        //drivetrain
        drive = new SampleMecanumDrive(hardwareMap);

        //trajectories are flipped direction

        if(spikeMark == 2) {
            //middle pixel trajectories
            buildMiddlePixelTraj();
        }

        //start
        waitForStart();

        //init attachments
        Arm.init();
        Claw.init();
        Claw.close();
        sleep(1000);

        if(spikeMark == 2) {
            followMiddlePixelTraj();
        }
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
        Claw.open();
        sleep(1000);

        //retract
        Arm.reset();
        Claw.reset();
        waitForArm();

        //park
        drive.followTrajectory(mTraj5);
        drive.followTrajectory(mTraj6);
    }

    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && !Arm.finishedDelivery()) {
            //wait
        }
        sleep(1000);
    }

}
