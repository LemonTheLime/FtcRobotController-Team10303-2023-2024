package org.firstinspires.ftc.teamcode.test.motortest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "TestAutonomous", group = "Motor Testing")
public class TestAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTraj = drive.trajectoryBuilder(new Pose2d())
                .back(20.5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTraj);
    }
}
