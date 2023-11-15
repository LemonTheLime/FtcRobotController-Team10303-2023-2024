package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ArmControl
 * runs the arm attachment of the robot
 */
public class ArmControl{

    //FIELDS
    //hardware map
    private HardwareMap hardwareMap = null;
    //arm status
    private boolean status;
    //telemetry
    private Telemetry t = null;
    //motor fields
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private int ticksPerRev = 288;
    private double power = 0.99;
    private int leftEncoderValue;
    private int rightEncoderValue;
    //rotation constants
    private double offset = 180.0; //offset angle (starting angle)
    private double rotation; //current angle in degrees, 0 is terminal x axis
    private double targetRotation = offset; //target rotation
    private double maxRotation = offset; //arm starts off here
    private double minRotation = -30.0;
    private double gearRatio = 32.0 / 10.0;
    private double deliverRotation = 38.19;

    //CONSTRUCTOR
    public ArmControl(HardwareMap hwMap, Telemetry t) {
        status = false;
        this.t = t;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize motor hardware
    private void initHardware() {
        //get motors from ids
        leftMotor = hardwareMap.get(DcMotor.class, "leftArm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightArm");

        //reverse motors here if needed:
        //arm rotating out should negative encoder changes for both motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //initialize arm control mechanism
    public void init() {
        status = true;
    }

    //telemetry
    public void telemetryOutput() {
        //telemetry
        getEncoderValues();
        t.addLine("ArmControl: ");
        t.addData("status", status);
        t.addData("leftEncoderValue", leftEncoderValue);
        t.addData("rightEncoderValue", rightEncoderValue);
        t.addData("rotation", rotation);
        t.addData("targetRotation", targetRotation);
        t.addLine();
    }

    //goes to target position
    public void goToTargetRotation(double angle) {
        if(status) {
            //get encoder targets
            getEncoderValues();
            int targetValue = (int)(ticksPerRev / 360.0 * (angle - offset) * gearRatio);
            t.addData("targetValue", targetValue);
            leftMotor.setTargetPosition(targetValue);
            rightMotor.setTargetPosition(targetValue);

            //set motors to run with position
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //power motors (move motor towards target position)
            //experimental power factor
            leftMotor.setPower(powerFunction(power));
            rightMotor.setPower(powerFunction(power));
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        leftEncoderValue = leftMotor.getCurrentPosition();
        rightEncoderValue = rightMotor.getCurrentPosition();
        rotation = offset + (double)leftEncoderValue / ticksPerRev * 360.0 / gearRatio;
    }

    //rotates the target arm angle
    public void rotate(double angle) {
        //set the targetRotation
        targetRotation += angle;
        if(targetRotation < minRotation) {
            targetRotation = minRotation;
        }
        if(targetRotation > maxRotation) {
            targetRotation = maxRotation;
        }

        //rotate the arm to the target
        goToTargetRotation(targetRotation);
    }

    //rotates arm to ground
    public void ground() {
        targetRotation = minRotation;
        goToTargetRotation(targetRotation);
    }

    //reset arm
    public void reset() {
        targetRotation = maxRotation;
        goToTargetRotation(targetRotation);
    }

    //toRadians
    private double toRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    //power function
    private double powerFunction(double rawPower) {
        double powerFactor = (1 - Math.sin(toRadians(rotation)));
        //powerFactor = 1;
        double minPower = 0.5;
        double maxPower = 0.75;
        double finalPower = rawPower * powerFactor * (maxPower - minPower) + minPower;
        return rawPower;
        //return (finalPower);
    }

    //reset the encoders
    public void resetEncoder() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //set to deliver rotation
    public void deliver() {
        targetRotation = deliverRotation;
        goToTargetRotation(targetRotation);
    }
}
