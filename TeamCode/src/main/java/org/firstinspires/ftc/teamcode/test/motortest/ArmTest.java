package org.firstinspires.ftc.teamcode.test.motortest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Autonomous
public class ArmTest extends LinearOpMode {

    private DcMotorEx motor = null;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "rightArm");

        waitForStart();
        motor.setPower(0.4);
        sleep(10000);
        motor.setPower(0);
    }
}
