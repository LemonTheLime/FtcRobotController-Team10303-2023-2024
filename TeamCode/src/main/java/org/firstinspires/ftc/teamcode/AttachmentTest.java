package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.attachments.LauncherControl;

@TeleOp(name="AttachmentTest", group = "Testing")
public class AttachmentTest extends OpMode {

    //FIELDS
    private ArmControl arm = null;
    private ClawControl claw = null;
    private LauncherControl launcher = null;

    @Override
    public void init() {
        arm = new ArmControl(hardwareMap, telemetry);
        claw = new ClawControl(hardwareMap, telemetry);
        launcher = new LauncherControl(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //attachments only give telemetry and are not initialized
        arm.telemetryOutput();
        claw.telemetryOutput();
        launcher.telemetryOutput();
        telemetry.update();
    }
}
