package org.firstinspires.ftc.teamcode.attachments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ClawControl
 * runs the servos of the claw attachment
 */
@Config
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
    private static final double LEFT_OPEN = 0.35;
    private static final double LEFT_CLOSE = 0.6;
    private static final double RIGHT_OPEN = 0.31;
    private static final double RIGHT_CLOSE = 0.56;
    //pitch constants
    private double pitch = 0; //position of pitch servo
    private static final double BACKDROP_ANGLE = 30.0;
    private static final double GROUND_PITCH = 0.713;
    private static final double DELIVER_PITCH = 0.794; //0.75 * (180 + BACKDROP_ANGLE - ArmControl.DELIVER_ROTATION)/(180 + BACKDROP_ANGLE - 40.0); //0.838
    private static final double AUTO_DELIVER_PITCH = 0.82;
    //states
    private ClawState clawState = ClawState.BOTH_CLOSE;
    private PitchState pitchState = PitchState.MANUAL;

    //config
    public static boolean testMode = false;
    public static double LEFT_OPEN_TEST = 0.35;
    public static double LEFT_CLOSE_TEST = 0.6;
    public static double RIGHT_OPEN_TEST = 0.31;
    public static double RIGHT_CLOSE_TEST = 0.56;

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
        t.addData("clawState", clawState);
        t.addData("pitchState", pitchState);
        t.addData("leftOpen", leftOpen);
        t.addData("rightOpen", rightOpen);
        t.addData("pitch", pitch);
        t.addLine();
    }

    //open left claw
    public void openLeftClaw() {
        if(status) {
            double angle = LEFT_OPEN;
            if(testMode) {
                angle = LEFT_OPEN_TEST;
            }
            leftClaw.setPosition(angle);
            leftOpen = true;
            updateClawState();
        }
    }

    //close left claw
    public void closeLeftClaw() {
        if(status) {
            double angle = LEFT_CLOSE;
            if(testMode) {
                angle = LEFT_CLOSE_TEST;
            }
            leftClaw.setPosition(angle);
            leftOpen = false;
            updateClawState();
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
            double angle = RIGHT_OPEN;
            if(testMode) {
                angle = RIGHT_OPEN_TEST;
            }
            rightClaw.setPosition(angle);
            rightOpen = true;
            updateClawState();
        }
    }

    //close right claw
    public void closeRightClaw() {
        if(status) {
            double angle = RIGHT_CLOSE;
            if(testMode) {
                angle = RIGHT_CLOSE_TEST;
            }
            rightClaw.setPosition(angle);
            rightOpen = false;
            updateClawState();
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

    //update the clawstate
    private void updateClawState() {
        if(leftOpen && rightOpen) {
            clawState = ClawState.BOTH_OPEN;
        }
        if(!leftOpen && !rightOpen) {
            clawState = ClawState.BOTH_CLOSE;
        }
        if(leftOpen && !rightOpen) {
            clawState = ClawState.LEFT_OPEN_ONLY;
        }
        if(!leftOpen && rightOpen) {
            clawState = ClawState.RIGHT_OPEN_ONLY;
        }
    }

    //pitch servo uses a finite state machine to be able to run with the motors
    //rotates the pitch servo
    public void rotate(double angle) {
        if(status) {
            if(angle != 0) {
                pitchState = PitchState.MANUAL;
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
    }

    //rotate pitch to a certain angle
    public void rotateTo(double angle) {
        if(status) {
            if (angle > 1) {
                pitchLeft.setPosition(1);
                pitchRight.setPosition(1);
            } else if (angle < 0) {
                pitchLeft.setPosition(0);
                pitchRight.setPosition(0);
            } else {
                pitchLeft.setPosition(angle);
                pitchRight.setPosition(angle);
            }
        }
    }

    //ground pitch
    public void ground() {
        if(status) {
            pitchState = PitchState.PRESET;
            pitch = GROUND_PITCH;
            rotateTo(pitch);
        }
    }

    //reset pitch for initial
    public void reset() {
        if(status) {
            pitchState = PitchState.PRESET;
            pitch = 0;
            rotateTo(pitch);
        }
    }

    //deliver pitch for teleop
    public void deliver() {
        if(status) {
            pitchState = PitchState.DELIVER;
            pitch = DELIVER_PITCH;
            rotateTo(pitch);
        }
    }

    //calculates the servo ratio for delivery adjustment
    private double calcServoRatio(double angle) {
        double pitchTolerance = 0.0;
        if(angle < 40) {
            return DELIVER_PITCH * (180 + BACKDROP_ANGLE - angle + pitchTolerance) / (180 + BACKDROP_ANGLE - ArmControl.DELIVER_ROTATION);
        } else {
            return DELIVER_PITCH;
        }
    }

    //deliver pitch for autonomous
    public void autoDeliver() {
        if(status) {
            pitchState = PitchState.AUTONOMOUS;
            pitch = AUTO_DELIVER_PITCH;
            rotateTo(pitch);
        }
    }

    //updates the claw with arm motor rotation
    public void update(double angle) {
        if(status) {
            if(pitchState == PitchState.DELIVER) {
                pitch = calcServoRatio(angle);
                rotateTo(pitch);
            }
        }
    }

    //enum claw states
    private enum ClawState {
        BOTH_CLOSE,
        BOTH_OPEN,
        LEFT_OPEN_ONLY,
        RIGHT_OPEN_ONLY
    }

    //enum pitch states
    private enum PitchState {
        MANUAL,
        PRESET,
        DELIVER,
        AUTONOMOUS
    }
}