package org.firstinspires.ftc.teamcode.attachments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Lift Control
 * runs the motor for the tape measure lifts
 */
@Config
public class LiftControl {

    //hardware
    private HardwareMap hardwareMap = null;
    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    //attachment status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor specific constants
    private static final int TICKS_PER_REV = 288;
    private static final double GEAR_RATIO = 1;
    //private double power = 0.2;
    public static double motorPower = 0.5;
    public static double servoPower = 1.0;
    private int leftEncoderValue;
    private int rightEncoderValue;
    //rotation constants
    private static final double MIN_ROTATION = 0; //starting angle
    private static final double MAX_ROTATION = 20 * 2 * Math.PI; //20 rotations, each rotation decreases length
    private double rotation; //current angle in degrees, 0 is terminal x axis
    //state
    private LiftControl.LiftState leftState = LiftControl.LiftState.REST;
    private LiftControl.LiftState rightState = LiftControl.LiftState.REST;


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
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder"); //leftLiftMotor
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        //reverse motors here if needed:
        //leftMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //get servos from ids
        leftServo = hardwareMap.get(CRServo.class, "leftLiftServo");
        rightServo = hardwareMap.get(CRServo.class,"rightLiftServo");

        //reverse servos here if needed:
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);

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
        t.addData("leftState", leftState);
        t.addData("rightState", rightState);
        t.addData("leftEncoderValue", leftEncoderValue);
        t.addData("rightEncoderValue", rightEncoderValue);
        t.addData("rotation", rotation);
        t.addLine();
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        if(status) {
            //leftEncoderValue = leftMotor.getCurrentPosition();
            leftEncoderValue = leftMotor.getCurrentPosition();
            rightEncoderValue = rightMotor.getCurrentPosition();

            rotation = MIN_ROTATION + (double) leftEncoderValue / TICKS_PER_REV * 360.0 / GEAR_RATIO;
        }
    }

    //TELEOP: check for the current lift state and perform respective actions
    public void update() {
        if(status) {
            //left lift
            switch (leftState) {
                case REST:
                    //do nothing
                    leftMotor.setPower(0);
                    leftServo.setPower(0);
                    break;
                case RAISE:
                    //raise the lift
                    leftMotor.setPower(motorPower);
                    leftServo.setPower(servoPower);
                    break;
                case LOWER:
                    //lower the lift
                    leftMotor.setPower(-2 * motorPower);
                    leftServo.setPower(-servoPower);
                    break;
            }
            //right lift
            switch (rightState) {
                case REST:
                    //do nothing
                    rightMotor.setPower(0);
                    rightServo.setPower(0);
                    break;
                case RAISE:
                    //raise the lift
                    rightMotor.setPower(motorPower);
                    rightServo.setPower(servoPower);
                    break;
                case LOWER:
                    //lower the lift
                    rightMotor.setPower(-2 * motorPower);
                    rightServo.setPower(-servoPower);
                    break;
            }
        }
    }

    //rest lift 0:left, 1:right
    public void rest(int side) {
        if(side == 0) {
            leftState = LiftState.REST;
        } else if(side == 1) {
            rightState = LiftState.REST;
        }
    }

    //raise lift 0:left, 1:right
    public void raise(int side) {
        if(side == 0) {
            leftState = LiftState.RAISE;
        } else if(side == 1) {
            rightState = LiftState.RAISE;
        }
    }

    //lower lift 0:left, 1:right
    public void lower(int side) {
        if(side == 0) {
            leftState = LiftState.LOWER;
        } else if(side == 1) {
            rightState = LiftState.LOWER;
        }
    }

    public enum LiftState {
        REST,
        RAISE,
        LOWER
    }
}
