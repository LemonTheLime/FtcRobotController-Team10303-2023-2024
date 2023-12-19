package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* LauncherControl
 * runs the motor for the drone launcher
 */
public class LauncherControl {

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //launcher status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor fields
    private Servo launcher;
    private boolean open = false;

    //constructor
    public LauncherControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize motor hardware
    private void initHardware() {
        //get motor from id
        launcher = hardwareMap.get(Servo.class, "launcher");

        //change direction if necessary
    }

    //initializes launcher mechanism
    public void init() {
        status = true;
    }

    //telemetry
    public void telemetryOutput() {
        t.addLine("LauncherControl: ");
        t.addData("status", status);
        t.addData("position", open);
        t.addLine();
    }

    //run the servo
    public void open() {
        if(status) {
            open = true;
            launcher.setPosition(0.2);
        }
    }

    //stop the servo
    public void close() {
        if(status) {
            open = false;
            launcher.setPosition(1);
        }
    }
}
