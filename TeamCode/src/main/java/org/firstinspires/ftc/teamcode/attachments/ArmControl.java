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
    private double power = 0.8;
    private int leftEncoderValue;
    private int rightEncoderValue;
    //rotation constants
    private double offset = 200; //offset angle (starting angle)
    private double rotation; //current angle in degrees, 0 is terminal x axis
    private double targetRotation = offset; //target rotation
    private double maxRotation = offset; //arm starts off here
    private double minRotation = -10;

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
            int targetValue = (int)(ticksPerRev / 360.0 * (angle - offset));
            t.addData("targetValue", targetValue);
            leftMotor.setTargetPosition(targetValue);
            rightMotor.setTargetPosition(targetValue);

            //set motors to run with position
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //power motors (move motor towards target position)
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        leftEncoderValue = leftMotor.getCurrentPosition();
        rightEncoderValue = rightMotor.getCurrentPosition();
        rotation = offset + (double)leftEncoderValue / ticksPerRev * 360.0;
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
}
