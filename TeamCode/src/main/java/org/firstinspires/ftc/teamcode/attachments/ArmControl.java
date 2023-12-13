package org.firstinspires.ftc.teamcode.attachments;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ArmControl
 * runs the arm attachment of the robot
 */

@Config
public class ArmControl {

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //attachment status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //hardware fields
    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;
    //motor specific constants      -trying hd hex motor
    private int ticksPerRev = 28;
    private double gearRatio = 32.0 / 10.0;
    private int armVelocity = 300; //ticks per second
    private double power = 1;
    private int leftEncoderValue;
    private int rightEncoderValue;
    //rotation constants
    private double offset = 180.0; //offset angle (starting angle)
    private double rotation; //current angle in degrees, 0 is terminal x axis
    private double targetRotation = offset; //target rotation
    private double maxRotation = offset; //arm starts off here
    private double minRotation = -30.0;
    private double deliverRotation = 40; //teleop
    private double autoDeliverRotation = 10; //autonomous

    private DcMotorEx exEncoder = null;
    private int exEncoderCount = 0;

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
        rightMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        rightMotor.setTargetPositionTolerance(tolerance);
    }

    //initialize motor hardware
    private void initHardware() {
        //get motors from ids
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightArm");

        //reverse motors here if needed:
        //arm rotating out should be negative encoder changes for both motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        exEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
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
        t.addData("external encoder", exEncoderCount);
        t.addLine();
    }

    //rotates arm to target position
    public void goToTargetRotation(double angle) {
        if(status) {
            //get encoder targets
            getEncoderValues();
            int targetValue = (int)(ticksPerRev / 360.0 * (angle - offset) * gearRatio);
            t.addData("targetValue", targetValue);

            //set motor targets
            leftMotor.setTargetPosition(targetValue);
            rightMotor.setTargetPosition(targetValue);

            //set motors to run with position
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //set motor velocities
            //leftMotor.setVelocity(armVelocity);
            //rightMotor.setVelocity(armVelocity);
            rightMotor.setPower(power);
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        if(status) {
            leftEncoderValue = leftMotor.getCurrentPosition();
            rightEncoderValue = rightMotor.getCurrentPosition();
            rotation = offset + (double) rightEncoderValue / ticksPerRev * 360.0 / gearRatio;
            exEncoderCount = exEncoder.getCurrentPosition();
        }
    }

    //rotates the target arm by an angle change
    public void rotate(double angle) {
        if(status) {
            //set the targetRotation
            targetRotation += angle;

            //arm having encoder static issues
            /*
            if (targetRotation < minRotation) {
                targetRotation = minRotation;
            }
            if (targetRotation > maxRotation) {
                targetRotation = maxRotation;
            }

             */

            //rotate the arm to the target
            goToTargetRotation(targetRotation);
        }
    }

    //preset arm rotation to ground
    public void ground() {
        if(status) {
            targetRotation = minRotation;
            goToTargetRotation(targetRotation);
        }
    }

    //preset arm rotation to the starting state
    public void reset() {
        if(status) {
            targetRotation = maxRotation;
            goToTargetRotation(targetRotation);
        }
    }

    //partially pull back arm for auto at 90 degrees
    public void armUp() {
        if(status) {
            targetRotation = 90;
            goToTargetRotation(targetRotation);
        }
    }

    //reset the encoders (arm should be at reset state)
    public void resetEncoder() {
        if(status) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            targetRotation = 180;
        }
    }

    //preset arm rotation for delivering pixel to backdrop in teleop
    public void deliver() {
        if(status) {
            targetRotation = deliverRotation;
            goToTargetRotation(targetRotation);
        }
    }

    //preset arm rotation for delivering pixel to backdrop in autonomous
    public void autoDeliver() {
        if(status) {
            targetRotation = autoDeliverRotation;
            goToTargetRotation(targetRotation);
        }
    }

    //waits until the arm has delivered for autonomous
    public boolean finishedDelivery() {
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
}
