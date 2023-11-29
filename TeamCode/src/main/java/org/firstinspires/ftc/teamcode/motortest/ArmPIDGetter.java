package org.firstinspires.ftc.teamcode.motortest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp
public class ArmPIDGetter extends OpMode {

    private DcMotorEx ArmMotor = null;
    private PIDFCoefficients c = null;

    @Override
    public void init() {
        ArmMotor = hardwareMap.get(DcMotorEx.class, "rightArm");
        c = ArmMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        telemetry.addData("PIDF Coefficients", c);
        telemetry.update();
    }
}
