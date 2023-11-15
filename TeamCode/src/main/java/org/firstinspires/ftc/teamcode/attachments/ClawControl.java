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
    private Servo pitchLeft = null; //controls the pitch of the claw on the arm
    private Servo pitchRight = null;
    //claw constants
    private boolean open;
    private double pitch = 0; //position of pitch servo
    private double groundPitch = 0.737;
    private double min = 0.85;
    private double max = 1.0;

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
        pitchLeft = hardwareMap.get(Servo.class, "leftPitch");
        pitchRight = hardwareMap.get(Servo.class, "rightPitch");

        //reverse direction
        pitchLeft.setDirection(Servo.Direction.REVERSE);
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
    public void open() {
        clawServo.setPosition(min);
    }

    //close the claw servo
    public void close() {
        clawServo.setPosition(max);
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
            rotateTo(pitch);
        }
    }

    //rotate to a certain angle
    public void rotateTo(double angle) {
        if(status) {
            if (pitch > 1) {
                pitch = 1;
            }
            if (pitch < 0) {
                pitch = 0;
            }

            pitchLeft.setPosition(pitch);
            pitchRight.setPosition(pitch);
        }
    }

    //ground pitch
    public void ground() {
        if(status) {
            pitch = groundPitch;
            pitchLeft.setPosition(pitch);
            pitchRight.setPosition(pitch);
        }
    }

    //reset pitch
    public void reset() {
        if(status) {
            pitch = 0;
            pitchLeft.setPosition(pitch);
            pitchRight.setPosition(pitch);
        }
    }
}
