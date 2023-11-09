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
    private ClawControl Claw = null;
    //private LauncherControl launcher = null;


    @Override
    public void init() {
        Arm = new ArmControl(hardwareMap, telemetry);
        Arm.init();
        Claw = new ClawControl(hardwareMap, telemetry);
        Claw.init();
        // = new LauncherControl(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //attachments only give telemetry and are not initialized
        Arm.telemetryOutput();
        Arm.goToTargetRotation(90);
        //Arm.rotate(-0.1);

        Claw.telemetryOutput();
        Claw.rotateTo(0.5);
        Claw.open();

        //launcher.telemetryOutput();
        telemetry.update();
    }
}
