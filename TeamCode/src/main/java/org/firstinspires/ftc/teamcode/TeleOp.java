package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;

public class TeleOp extends OpMode {

    //FIELDS

    //arm
    private ArmControl Arm = null;


    @Override
    public void init() {
        //get hardware for the arm
        Arm = new ArmControl(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

    }
}
