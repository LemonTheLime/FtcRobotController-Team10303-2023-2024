package org.firstinspires.ftc.teamcode.opmodes.AutonomousParkOnly;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//this program starts on the red right side and only parks in the corner
@Autonomous(name = "ParkOnlyRedRight", group = "Park Only")
public class RedRightAutoParkOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //build roadrunner paths
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //trajectories are flipped direction
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(3)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(48)
                .build();

        waitForStart();

        drive.followTrajectory(traj1);

        sleep(500);

        drive.followTrajectory(traj2);



    }
}
