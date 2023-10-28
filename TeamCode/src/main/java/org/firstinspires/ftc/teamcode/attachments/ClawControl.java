package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ClawControl
 * runs the servos of the claw attachment
 */
public class ClawControl{

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //claw status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //servo fields
    private Servo clawServo = null;
    private Servo pitchServo = null; //controls the pitch of the claw on the arm
    //claw constants
    private boolean open;
    private double pitch = 0; //position of pitch servo

    //CONSTRUCTOR
    public ClawControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize servo hardware
    private void initHardware() {
        //get servo from id
        clawServo = hardwareMap.get(Servo.class, "claw");
        pitchServo = hardwareMap.get(Servo.class, "pitch");

        //reverse direction
    }

    //initialize claw control mechanism
    public void init() {
        status = true;
    }

    //telemetry
    public void telemetryOutput() {
        //telemetry
        t.addLine("ClawControl: ");
        t.addData("status", status);
        t.addData("open", open);
        t.addData("pitch", pitch);
        t.addLine();
    }


    //open the claw servo
    private void open() {
        clawServo.setPosition(1);
    }

    //close the claw servo
    private void close() {
        clawServo.setPosition(0);
    }

    //change the claw servo position
    public void changeClaw() {
        if(status) {
            open = !open;
            if (open) {
                open();
            } else {
                close();
            }
        }
    }

    //pitch servo uses a finite state machine to be able to run with the motors
    //rotates the pitch servo
    public void rotate(double angle) {
        if(status) {
            //magnitude of angle determines speed of pitch change
            //speed pitch change cannot be to large or else arm will stutter

            //change pitch angle
            pitch += angle;
            if (pitch > 1) {
                pitch = 1;
            }
            if (pitch < 0) {
                pitch = 0;
            }

            //rotate servo to pitch angle
            pitchServo.setPosition(pitch);
        }
    }
}
