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
    //private DcMotorEx leftMotor = null; deprecate left arm
    private DcMotorEx armMotor = null;
    //attachment status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor specific constants
    private int ticksPerRev = (int)560;
    private double gearRatio = 32.0 / 10.0 * 0.9;
    private double power = 0.4;
    private int leftEncoderValue;
    private int rightEncoderValue;
    //rotation constants
    private double offset = 180.0; //starting angle
    private double rotation; //current angle in degrees, 0 is terminal x axis
    private double targetRotation = offset; //target rotation
    private double pastTargetRotation = offset;
    //preset angles
    private final double MAX_ROTATION = 180.0; //arm starts off here
    private final double MIN_ROTATION = -38.0;
    private final double DELIVER_ROTATION = 40; //teleop
    private final double AUTO_DELIVER_ROTATION = 10; //autonomous
    //state
    private ArmState state = ArmState.INITIAL;
    private boolean goingToGround = false; //ground call field

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
        //leftMotor = hardwareMap.get(DcMotorEx.class, "leftArm");
        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");

        //reverse motors here if needed:
        //arm rotating out should be negative encoder changes for both motors
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        t.addData("leftEncoderValue", leftEncoderValue);
        t.addData("rightEncoderValue", rightEncoderValue);
        t.addData("rotation", rotation);
        t.addData("targetRotation", targetRotation);
        t.addLine();
    }

    //rotates arm to target position
    private void goToTargetRotation(double angle) {
        if(status) {
            if(angle != pastTargetRotation) {
                //get encoder targets
                getEncoderValues();
                int targetValue = (int) (ticksPerRev / 360.0 * (angle - offset) * gearRatio);
                t.addData("targetValue", targetValue);

                //set motor targets
                //leftMotor.setTargetPosition(targetValue);
                armMotor.setTargetPosition(targetValue);

                //set motors to run with position
                //leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set power
                switch (state) {
                    case MOVING_TO_GROUND1:
                    case MOVING_TO_INITIAL1:
                        power = 0.4;
                        break;
                    case MOVING_TO_GROUND2:
                    case MOVING_TO_INITIAL2:
                        power = 0.2;
                        break;
                    case GROUND:
                    case INITIAL:
                        power = 0.0;
                        break;
                    case DELIVER:
                    case AUTO:
                        power = 0.3;
                        break;
                }

                armMotor.setPower(power);
            }

            //update past targetRotation
            pastTargetRotation = angle;
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        if(status) {
            //leftEncoderValue = leftMotor.getCurrentPosition();
            rightEncoderValue = armMotor.getCurrentPosition();
            rotation = offset + (double) rightEncoderValue / ticksPerRev * 360.0 / gearRatio;
        }
    }

    //TELEOP: set arm state to ground preset
    public void ground() {
        if(status) {
            if(state != ArmState.GROUND && state != ArmState.MOVING_TO_GROUND1
            && state != ArmState.MOVING_TO_GROUND2) {
                if(rotation > 0) {
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
                if(rotation > 0) {
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
            if(state != ArmState.DELIVER && state != ArmState.MOVING_TO_DELIVER) {
                state = ArmState.DELIVER;
            }
            targetRotation = DELIVER_ROTATION;
            goToTargetRotation(targetRotation);
        }
    }

    //TELEOP: reset the encoders (arm should be at reset state)
    public void resetEncoder() {
        if(status) {
            //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            targetRotation = 180;
        }
    }

    //TELEOP: rotates the target arm by an angle change
    public void rotate(double angle) {
        if(status) {
            //set the targetRotation
            targetRotation += angle;

            //arm boundaries
            if (targetRotation < MIN_ROTATION) {
                targetRotation = MIN_ROTATION;
            }
            if (targetRotation > MAX_ROTATION) {
                targetRotation = MAX_ROTATION;
            }

            //rotate the arm to the target if there is change
            if(angle != 0) {
                state = ArmState.MOVING_TO_CUSTOM;
                goToTargetRotation(targetRotation);
            }
        }
    }

    //TELEOP: check for the current arm state and perform respective actions
    public void update() {
        double tolerance = 5;
        getEncoderValues();
        switch (state) {
            case MOVING_TO_GROUND1:
                targetRotation = 0;
                goToTargetRotation(targetRotation);
                if(rotation < (0 + tolerance)) {
                    state = ArmState.MOVING_TO_GROUND2;
                }
                break;
            case MOVING_TO_GROUND2:
                targetRotation = MIN_ROTATION;
                goToTargetRotation(targetRotation);
                if(rotation < (MIN_ROTATION + tolerance)) {
                    state = ArmState.GROUND;
                }
                break;
            case MOVING_TO_INITIAL1:
                targetRotation = 90;
                goToTargetRotation(targetRotation);
                if(rotation > (90 - tolerance)) {
                    state = ArmState.MOVING_TO_INITIAL2;
                }
                break;
            case MOVING_TO_INITIAL2:
                targetRotation = MAX_ROTATION;
                goToTargetRotation(targetRotation);
                if(rotation > (MAX_ROTATION - tolerance)) {
                    state = ArmState.INITIAL;
                }
                break;
            case MOVING_TO_DELIVER:
                targetRotation = DELIVER_ROTATION;
                goToTargetRotation(targetRotation);
                if(rotation > (DELIVER_ROTATION - tolerance)) {
                    state = ArmState.DELIVER;
                }
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

    //AUTONOMOUS: waits until the arm has delivered
    public boolean autoFinishedDelivery() {
        if(status) {
            double tolerance = 10; //degree tolerance
            getEncoderValues();
            if (Math.abs(rotation - targetRotation) < tolerance) {
                return true;
            }
            return false;
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
