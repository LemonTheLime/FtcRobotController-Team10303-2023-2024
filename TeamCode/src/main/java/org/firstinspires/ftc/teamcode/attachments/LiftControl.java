package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Lift Control
 * runs the motor for the tape measure lifts
 */
public class LiftControl {

    //hardware
    private HardwareMap hardwareMap = null;
    private DcMotorEx leftLift = null;
    private DcMotorEx rightLift = null;
    //attachment status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor specific constants
    private static final int TICKS_PER_REV = 560;
    private static final double GEAR_RATIO = 1;
    private double power = 1;
    private int leftEncoderValue;
    private int rightEncoderValue;
    private double targetRotation;
    private double pastTargetRotation;
    //rotation constants
    private static final double MIN_ROTATION = 0; //starting angle
    private static final double MAX_ROTATION = 7 * 2 * Math.PI; //7 rotations, each rotation decreases length
    private double rotation; //current angle in degrees, 0 is terminal x axis
    //state
    private LiftControl.LiftState state = LiftControl.LiftState.INITIAL;

    //constructor
    public LiftControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize motor hardware
    private void initHardware() {
        //get motors from ids
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        //reverse motors here if needed:
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //initialize arm control mechanism
    public void init() {
        status = true;
    }

    //telemetry
    public void telemetryOutput() {
        getEncoderValues();
        t.addLine("LiftControl: ");
        t.addData("status", status);
        t.addData("state", state);
        t.addData("leftEncoderValue", leftEncoderValue);
        t.addData("rightEncoderValue", rightEncoderValue);
        t.addData("rotation", rotation);
        t.addData("targetRotation", targetRotation);
        t.addLine();
    }

    //rotates arm to a target position
    private void goToTargetRotation(double angle) {
        if(status) {
            //arm will not repeatedly set the target to the same value
            if(angle != pastTargetRotation) {
                //get encoder targets
                getEncoderValues();
                int targetValue = (int) (TICKS_PER_REV / 360.0 * (angle - MIN_ROTATION) * GEAR_RATIO);
                t.addData("targetValue", targetValue);

                //set motor targets
                leftLift.setTargetPosition(targetValue);
                rightLift.setTargetPosition(targetValue);

                //set motors to run with position
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                //set power
                switch (state) {
                    case MOVING_TO_CUSTOM:
                        power = 0.1;
                        break;
                    case INITIAL:
                        power = 0;
                        break;
                    case PULL:
                    case FINAL:
                        power = 1.0;
                        break;
                }

                leftLift.setPower(power);
                rightLift.setPower(power);
            }

            //update past targetRotation
            pastTargetRotation = angle;
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        if(status) {
            //leftEncoderValue = leftMotor.getCurrentPosition();
            leftEncoderValue = leftLift.getCurrentPosition();
            rightEncoderValue = rightLift.getCurrentPosition();

            rotation = MIN_ROTATION + (double) leftEncoderValue / TICKS_PER_REV * 360.0 / GEAR_RATIO;
        }
    }

    //rotates the target arm by an angle change
    public void rotate(double angle) {
        if(status) {
            //rotate the arm to the target if there is change
            if(angle != 0) {
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

                state = LiftControl.LiftState.MOVING_TO_CUSTOM;
            }
        }
    }

    //TELEOP: check for the current arm state and perform respective actions
    public void update() {
        double tolerance = 5;
        getEncoderValues();
        switch (state) {
            case INITIAL:
                targetRotation = MIN_ROTATION;
                goToTargetRotation(targetRotation);
                break;
            case MOVING_TO_CUSTOM:
                if(Math.abs(rotation - targetRotation) < tolerance) {
                    state = LiftControl.LiftState.CUSTOM;
                } else {
                    goToTargetRotation(targetRotation); //target rotation already changed in rotate()
                }
                break;
            case CUSTOM:
            case PULL:
            case FINAL:
                goToTargetRotation(targetRotation); //target rotation already changed in rotate()
                break;

        }
    }

    public enum LiftState {
        INITIAL,
        MOVING_TO_CUSTOM,
        CUSTOM,
        PULL,
        FINAL
    }
}
