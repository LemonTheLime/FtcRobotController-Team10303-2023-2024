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
    public void runOpMode() throws InterruptedException {

        //attachments
        Arm = new ArmControl(hardwareMap, telemetry);
        Claw = new ClawControl(hardwareMap, telemetry);

        //build roadrunner paths
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //trajectories are flipped direction
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(27.5)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(10)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .back(28.5)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(10)
                .build();
        //after the delivery
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeLeft(25)
                .build();



        //start
        waitForStart();

        //init attachments
        Arm.init();
        Claw.init();

        //clamp the claw
        Claw.close();
        sleep(1000);

        //follow the trajectories
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

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
        drive.followTrajectory(traj5);


    }

    //wait for the arm and claw to deliver
    private void waitForArm() {
        while(opModeIsActive() && !Arm.finishedDelivery()) {
            //wait
        }
        sleep(1000);
    }
}
