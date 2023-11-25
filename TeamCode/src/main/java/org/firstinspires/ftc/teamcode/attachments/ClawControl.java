package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ClawControl
 * runs the servos of the claw attachment
 */
public class ClawControl {

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //attachment status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //hardware fields
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo pitchLeft = null; //controls the pitch of the claw on the arm
    private Servo pitchRight = null;
    //claw constants
    private boolean leftOpen = false;
    private boolean rightOpen = false;
    private double leftMin = 0.0;
    private double leftMax = 1.0;
    private double rightMin = 0.;
    private double rightMax = 1.0;
    //pitch constants
    private double pitch = 0; //position of pitch servo
    private double groundPitch = 0.737;
    private double deliverPitch = 0.6;
    private double autoDeliverPitch = 0.82;

    //constructor
    public ClawControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize servo hardware
    private void initHardware() {
        //get servo from id
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        pitchLeft = hardwareMap.get(Servo.class, "leftPitch");
        pitchRight = hardwareMap.get(Servo.class, "rightPitch");

        //reverse direction
        pitchLeft.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);
    }

    //initialize claw control mechanism
    public void init() {
        status = true;
    }

    //telemetry
    public void telemetryOutput() {
        t.addLine("ClawControl: ");
        t.addData("status", status);
        t.addData("leftOpen", leftOpen);
        t.addData("rightOpen", rightOpen);
        t.addData("pitch", pitch);
        t.addLine();
    }

    //open left claw
    public void openLeftClaw() {
        if(status) {
            leftClaw.setPosition(leftMin);
            leftOpen = true;
        }
    }

    //close left claw
    public void closeLeftClaw() {
        if(status) {
            leftClaw.setPosition(leftMax);
            leftOpen = false;
        }
    }

    //change left claw
    public void changeLeft() {
        if(status) {
            leftOpen = !leftOpen;
            if (leftOpen) {
                openLeftClaw();
            } else {
                closeLeftClaw();
            }
        }
    }

    //open right claw
    public void openRightClaw() {
        if(status) {
            rightClaw.setPosition(rightMin);
            rightOpen = true;
        }
    }

    //close right claw
    public void closeRightClaw() {
        if(status) {
            rightClaw.setPosition(rightMax);
            rightOpen = false;
        }
    }

    //change right claw
    public void changeRight() {
        if(status) {
            rightOpen = !rightOpen;
            if (rightOpen) {
                openRightClaw();
            } else {
                closeRightClaw();
            }
        }
    }

    //change both claws
    //if both claws in same state, reverse both
    //if one open and other close, close both
    public void changeBothClaw() {
        if(status) {
            if(leftOpen && rightOpen) {
                //if both open, close both
                closeLeftClaw();
                closeRightClaw();
            } else if(!leftOpen && !rightOpen) {
                //if both close, open both
                openLeftClaw();
                openRightClaw();
            } else {
                //close both
                closeLeftClaw();
                closeRightClaw();
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

    //rotate pitch to a certain angle
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

    //deliver pitch for teleop
    public void deliver() {
        if(status) {
            pitch = deliverPitch;
            pitchLeft.setPosition(pitch);
            pitchRight.setPosition(pitch);
        }
    }

    //deliver pitch for autonomous
    public void autoDeliver() {
        if(status) {
            pitch = autoDeliverPitch;
            pitchLeft.setPosition(pitch);
            pitchRight.setPosition(pitch);
        }
    }
}