package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* LauncherControl
 * runs the motor for the drone launcher
 */
public class LauncherControl{

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //launcher status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor fields
    private DcMotor launcher;
    private double power = 1.0;
    private boolean running;

    //CONSTRUCTOR
    public LauncherControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize motor hardware
    private void initHardware() {
        //get motor from id
        launcher = hardwareMap.get(DcMotor.class, "launcher");

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
        if(status) {
            running = true;
            launcher.setPower(power);
        }
    }

    //stop the servo
    public void stop() {
        if(status) {
            running = false;
            launcher.setPower(0);
        }
    }

}
