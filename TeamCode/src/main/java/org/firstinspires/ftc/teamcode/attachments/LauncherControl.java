package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* LauncherControl
 * runs the continuous servo for the drone launcher
 */
public class LauncherControl{

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //launcher status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //servo fields
    private CRServo servo;
    private double power = 1.0;
    private boolean running;

    //CONSTRUCTOR
    public LauncherControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize CRServo hardware
    private void initHardware() {
        //get servo from id
        servo = hardwareMap.get(CRServo.class, "launcher");

        //change direction if necessary
    }

    //initializes launcher mechanism
    public void init() {
        status = true;
        running = false;
    }

    //telemetry
    public void telemetryOutput() {
        //telemetry
        t.addLine("LauncherControl: ");
        t.addData("status", status);
        t.addData("running", running);
        t.addLine();
    }

    //run the servo
    public void run() {
        running = true;
        servo.setPower(power);
    }

    //stop the servo
    public void stop() {
        running = false;
        servo.setPower(power);
    }

}
