package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;

public class TeleOp extends OpMode {

    //FIELDS

    //arm
    private ArmControl Arm = null;


    @Override
    public void init() {
        //create arm control mechanism
        Arm = new ArmControl(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

    }
}
