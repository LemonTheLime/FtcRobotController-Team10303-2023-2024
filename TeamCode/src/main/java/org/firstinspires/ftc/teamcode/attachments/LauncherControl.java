package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//runs the servos of the claw attachment
public class LauncherControl{

    //FIELDS
    //Hardware map
    private HardwareMap hardwareMap = null;
    //Servo fields
    private CRServo servo;
    private boolean status;
    private double power;
    private boolean running;

    //CONSTRUCTOR
    /* Opmodes will create an instance of ClawControl class to run the claw servo
     * takes in the hardwaremap
     */
    public LauncherControl(HardwareMap hwMap) {
        status = false;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize servo
    private void initHardware() {
        //get servo from id
        servo = hardwareMap.get(CRServo.class, "launcher");

        //change direction if necessary
    }

    //telemetry
    public void telemetryOutput(Telemetry t) {
        //telemetry
        t.addLine("LauncherControl: ");
        t.addData("status", status);
        t.addLine();
    }

    //initializes the claw control mechanism
    public void init() {
        status = true;
        running = false;
    }

    //run the servo
    public void run() {
        servo.setPower(power);
    }

    //stop the servo
    public void stop() {
        servo.setPower(power);
    }

}
