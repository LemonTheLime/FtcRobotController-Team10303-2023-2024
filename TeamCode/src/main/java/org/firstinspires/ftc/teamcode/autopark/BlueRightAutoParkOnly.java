package org.firstinspires.ftc.teamcode.autopark;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//this program starts on the blue right side and only parks on the other side of the backstage
@Autonomous(name="BlueRightAutoParkOnly")
public class BlueRightAutoParkOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //build roadrunner paths
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //trajectories are flipped direction
        Pose2d startPose = new Pose2d();

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .back(51)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .back(86).build();

        waitForStart();

        drive.followTrajectory(traj1);

        sleep(500);

        drive.turn(Math.toRadians(90));

        sleep(500);

        drive.followTrajectory(traj2);

    }
}
