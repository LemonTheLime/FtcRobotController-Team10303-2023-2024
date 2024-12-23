package org.firstinspires.ftc.teamcode.attachments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ArmControl
 * runs the arm attachment of the robot
 */

@Config
public class ArmControl {

    //hardware
    private HardwareMap hardwareMap = null;
    private DcMotorEx armMotor = null;
    //attachment status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor specific constants
    private final int TICKS_PER_REV = 560;
    private final double GEAR_RATIO = 32.0 / 10.0 * 0.9;
    private double power = 0.4;
    private int armEncoderValue;
    //rotation constants
    private double offset = 180.0; //starting angle
    private double rotation; //current angle in degrees, 0 is terminal x axis
    private double targetRotation = offset; //target rotation
    private double pastTargetRotation = offset;
    //preset angles
    private final double MAX_ROTATION = 180.0; //arm starts off here
    private final double MIN_ROTATION = -38.0;
    private final double DELIVER_ROTATION = 40; //teleop
    private final double AUTO_DELIVER_ROTATION = 0; //autonomous
    //state
    private ArmState state = ArmState.INITIAL;

    //config
    public static double kP = 10;
    public static double kI = 0.05;
    public static double kD = 0;
    public static double kF = 0;
    public static int tolerance = 5;

    //constructor
    public ArmControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //updates PIDF coefficients
    public void updatePIDF() {
        armMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        armMotor.setTargetPositionTolerance(tolerance);
    }

    //initialize motor hardware
    private void initHardware() {
        //get motors from ids
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        //reverse motors here if needed:
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //initialize arm control mechanism
    public void init() {
        status = true;
    }

    //telemetry
    public void telemetryOutput() {
        getEncoderValues();
        t.addLine("ArmControl: ");
        t.addData("status", status);
        t.addData("state", state);
        t.addData("armEncoderValue", armEncoderValue);
        t.addData("rotation", rotation);
        t.addData("targetRotation", targetRotation);
        t.addLine();
    }

    //rotates arm to a target position
    private void goToTargetRotation(double angle) {
        if(status) {
            //get encoder targets
            getEncoderValues();
            int targetValue = (int) (TICKS_PER_REV / 360.0 * (angle - offset) * GEAR_RATIO);
            t.addData("targetValue", targetValue);

            //arm will not repeatedly set the target to the same value
            if(angle != pastTargetRotation) {
                //set motor targets
                armMotor.setTargetPosition(targetValue);

                //set motors to run with position
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set power
                switch (state) {
                    case MOVING_TO_GROUND1:
                        power = 0.4;
                        break;
                    case MOVING_TO_INITIAL1:
                        power = 0.6;
                        break;
                    case MOVING_TO_GROUND2:
                    case MOVING_TO_INITIAL2:
                        power = 0.2;
                        break;
                    case GROUND:
                    case INITIAL:
                        power = 0.0;
                        break;
                    case MOVING_TO_DELIVER:
                    case DELIVER:
                    case MOVING_TO_CUSTOM:
                    case CUSTOM:
                    case AUTO:
                        power = 0.25;
                        break;
                }

                armMotor.setPower(power);
            } else {
                if(state == ArmState.INITIAL || state == ArmState.GROUND) {
                    //set motor targets
                    armMotor.setTargetPosition(targetValue);

                    //set motors to run with position
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //zero power
                    power = 0;
                    armMotor.setPower(power);
                }
            }

            //update past targetRotation
            pastTargetRotation = angle;
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        if(status) {
            //leftEncoderValue = leftMotor.getCurrentPosition();
            armEncoderValue = armMotor.getCurrentPosition();
            rotation = offset + (double) armEncoderValue / TICKS_PER_REV * 360.0 / GEAR_RATIO;
        }
    }

    //TELEOP: set arm state to ground preset
    public void ground() {
        if(status) {
            if(state == ArmState.INITIAL) {
                resetEncoder();
            }
            if(state != ArmState.GROUND && state != ArmState.MOVING_TO_GROUND1
            && state != ArmState.MOVING_TO_GROUND2) {
                if(rotation > 90) {
                    state = ArmState.MOVING_TO_GROUND1;
                } else {
                    state = ArmState.MOVING_TO_GROUND2;
                }
            }
        }
    }

    //TELEOP: set arm state to initial preset
    public void reset() {
        if(status) {
            if(state != ArmState.INITIAL && state != ArmState.MOVING_TO_INITIAL1
                    && state != ArmState.MOVING_TO_INITIAL2) {
                if(rotation < 0) {
                    state = ArmState.MOVING_TO_INITIAL1;
                } else {
                    state = ArmState.MOVING_TO_INITIAL2;
                }
            }
        }
    }

    //TELEOP: preset arm rotation for delivering pixel to backdrop
    public void deliver() {
        if(status) {
            if(state == ArmState.INITIAL) {
                resetEncoder();
            }
            if(state != ArmState.DELIVER && state != ArmState.MOVING_TO_DELIVER) {
                state = ArmState.MOVING_TO_DELIVER;
            }
        }
    }

    //TELEOP: reset the encoders (arm should be at initial state)
    public void resetEncoder() {
        if(status) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            switch (state) {
                case INITIAL:
                    offset = 180.0;
                    targetRotation = offset;
                    break;
                case GROUND:
                    offset = -36.0;
                    targetRotation = offset;
            }

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //TELEOP: rotates the target arm by an angle change
    public void rotate(double angle) {
        if(status) {
            //rotate the arm to the target if there is change
            if(angle != 0) {
                if(state == ArmState.INITIAL) {
                    resetEncoder();
                }
                //set the targetRotation to current angle + change in angle
                getEncoderValues();
                targetRotation = rotation + angle;

                //arm boundaries
                if (targetRotation < MIN_ROTATION) {
                    targetRotation = MIN_ROTATION;
                }
                if (targetRotation > MAX_ROTATION) {
                    targetRotation = MAX_ROTATION;
                }

                state = ArmState.MOVING_TO_CUSTOM;
            } else {
                if(state == ArmState.MOVING_TO_CUSTOM) {
                    state = ArmState.CUSTOM;
                }
            }
        }
    }

    //TELEOP: check for the current arm state and perform respective actions
    public void update() {
        double tolerance = 2.5;
        getEncoderValues();
        switch (state) {
            case MOVING_TO_GROUND1:
                if(rotation < (90 + tolerance)) {
                    state = ArmState.MOVING_TO_GROUND2;
                } else {
                    targetRotation = 0;
                    goToTargetRotation(targetRotation);
                }
                break;
            case MOVING_TO_GROUND2:
                if(rotation < (MIN_ROTATION + tolerance)) {
                    state = ArmState.GROUND;
                } else {
                    targetRotation = MIN_ROTATION;
                    goToTargetRotation(targetRotation);
                }
                break;
            case GROUND:
                targetRotation = MIN_ROTATION;
                goToTargetRotation(targetRotation);
                break;
            case MOVING_TO_INITIAL1:
                if(rotation > (90 - tolerance)) {
                    state = ArmState.MOVING_TO_INITIAL2;
                } else {
                    targetRotation = 90;
                    goToTargetRotation(targetRotation);
                }
                break;
            case MOVING_TO_INITIAL2:
                if(rotation > (MAX_ROTATION - tolerance)) {
                    state = ArmState.INITIAL;
                } else {
                    targetRotation = MAX_ROTATION;
                    goToTargetRotation(targetRotation);
                }
                break;
            case INITIAL:
                targetRotation = MAX_ROTATION;
                goToTargetRotation(targetRotation);
                break;
            case MOVING_TO_DELIVER:
                if(rotation > (DELIVER_ROTATION - tolerance)) {
                    state = ArmState.DELIVER;
                } else {
                    targetRotation = DELIVER_ROTATION;
                    goToTargetRotation(targetRotation);
                }
                break;
            case DELIVER:
                targetRotation = DELIVER_ROTATION;
                goToTargetRotation(targetRotation);
                break;
            case MOVING_TO_CUSTOM:
            case CUSTOM:
                goToTargetRotation(targetRotation); //target rotation already changed in rotate()
                break;
        }
    }

    //AUTONOMOUS: partially pull back arm at 90 degrees
    public void autoArmUp() {
        if(status) {
            state = ArmState.AUTO;
            targetRotation = 90;
            goToTargetRotation(targetRotation);
        }
    }

    //AUTONOMOUS: set arm state to initial preset
    public void autoReset() {
        if(status) {
            state = ArmState.AUTO;
            targetRotation = 180;
            goToTargetRotation(targetRotation);
        }
    }

    //AUTONOMOUS: preset arm rotation for delivering pixel to backdrop
    public void autoDeliver() {
        if(status) {
            state = ArmState.AUTO;
            targetRotation = AUTO_DELIVER_ROTATION;
            goToTargetRotation(targetRotation);
        }
    }

    //AUTONOMOUS: checks if the arm motor is busy
    public boolean autoIsBusy() {
        if(status) {
            double tolerance = 3; //degree tolerance
            getEncoderValues();
            return Math.abs(rotation - targetRotation) > tolerance;
        }
        return false;
    }

    //enum arm states
    public enum ArmState {
        INITIAL,
        MOVING_TO_INITIAL1,
        MOVING_TO_INITIAL2,
        GROUND,
        MOVING_TO_GROUND1,
        MOVING_TO_GROUND2,
        CUSTOM,
        MOVING_TO_CUSTOM,
        DELIVER,
        MOVING_TO_DELIVER,
        AUTO
    }
}
