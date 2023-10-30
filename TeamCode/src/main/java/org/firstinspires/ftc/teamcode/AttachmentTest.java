package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.attachments.LauncherControl;

@TeleOp(name="AttachmentTest", group = "Testing")
public class AttachmentTest extends OpMode {

    //FIELDS
    private ArmControl Arm = null;
    //private ClawControl claw = null;
    //private LauncherControl launcher = null;


    @Override
    public void init() {
        Arm = new ArmControl(hardwareMap, telemetry);
        Arm.init();
        //claw = new ClawControl(hardwareMap, telemetry);
        // = new LauncherControl(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //attachments only give telemetry and are not initialized
        Arm.telemetryOutput();
        //Arm.goToTargetRotation(0);
        Arm.rotate(-0.05);

        //claw.telemetryOutput();
        //launcher.telemetryOutput();
        telemetry.update();
    }
}
