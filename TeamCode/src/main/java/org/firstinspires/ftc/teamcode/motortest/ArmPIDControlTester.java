package org.firstinspires.ftc.teamcode.motortest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.attachments.ArmPIDControl;

@Config
@Autonomous
public class ArmPIDControlTester extends LinearOpMode {

    private ArmPIDControl Arm = null;
    public static double rotation = 45;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm = new ArmPIDControl(hardwareMap, telemetry);
        Arm.init();

        waitForStart();

        while(opModeIsActive()) {
            sleep(1000);
            Arm.goToTargetRotation(rotation);


            //update pidf if needed
            if(gamepad1.a) {
                Arm.updatePIDF();
            }
        }
    }

}
