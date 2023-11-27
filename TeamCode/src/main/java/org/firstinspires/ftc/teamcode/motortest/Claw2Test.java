package org.firstinspires.ftc.teamcode.motortest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Claw2Test", group = "Motor Testing")
public class Claw2Test extends OpMode {

    //fields
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private ElapsedTime time = null;
    private boolean clawOpen = false;

    public void init() {
        //get hardware
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        //reverse servos if necessary
        leftClaw.setDirection(Servo.Direction.REVERSE);

        //time
        time = new ElapsedTime();

    }


    public void loop() {
        //telemetry
        telemetry.addData("time", time);
        telemetry.addLine("Open and close every 2 second.");
        telemetry.update();

        //loop
        if(time.seconds() > 2) {
            time.reset();
            if(clawOpen) {
                closeClaw();
            } else {
                openClaw();
            }
            clawOpen = !clawOpen;
        }
    }

    //opens the claw
    private void openClaw() {
        leftClaw.setPosition(1);
        rightClaw.setPosition(1);
    }

    //close the claw
    private void closeClaw() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
    }
}
