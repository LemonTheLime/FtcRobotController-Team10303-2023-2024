package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//runs the motors of the arm attachment
public class ArmControl{

    //FIELDS
    //Hardware map
    private HardwareMap hardwareMap = null;
    //Declare motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    //Arm status
    private boolean status;
    //Motor Constants:
    private int ticksPerRev;
    private double power = 0.8;
    //Time
    private ElapsedTime timer = null;
    //Encoder counts
    private int leftEncoderValue;
    private int rightEncoderValue;
    //Rotation constants
    private double offset = 200; //offset angle (starting angle)
    private double rotation; //current angle in degrees, 0 is terminal x axis
    private double targetRotation = offset; //target rotation
    private double maxRotation = offset; //arm starts off here
    private double minRotation = -10;

    //CONSTRUCTOR
    /* Opmodes will create an instance of ArmControl class to run the arm motors
     * takes in the hardwaremap
     */
    public ArmControl(HardwareMap hwMap) {
        status = false;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize motors
    private void initHardware() {
        //get motors from ids
        leftMotor = hardwareMap.get(DcMotor.class, "leftArm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightArm");

        //reverse motors here if needed:
        //arm rotating out should negative encoder changes for both motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        //set motors to run with encoders to target positions
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //telemetry
    public void telemetryOutput(Telemetry t) {
        //telemetry
        getEncoderValues();
        t.addLine("ArmControl: ");
        t.addData("status", status);
        t.addData("leftEncoderCount", leftEncoderValue);
        t.addData("rightEncoderCount", rightEncoderValue);
        t.addData("rotation", rotation);
        t.addData("targetRotation", targetRotation);
        t.addLine();
    }

    //initializes the arm control mechanism
    public void init() {
        status = true;
    }

    //goes to target position
    public void goToTargetRotation(double angle) {
        if(status) {
            //get encoder targets
            getEncoderValues();
            int targetValue = (int)(ticksPerRev / 360 * (angle - offset));
            leftMotor.setTargetPosition(targetValue);
            rightMotor.setTargetPosition(targetValue);

            //power motors (move motor towards target position)
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

    //gets encoder values from motors and calculates rotation
    private void getEncoderValues() {
        leftEncoderValue = leftMotor.getCurrentPosition();
        rightEncoderValue = rightMotor.getCurrentPosition();
        rotation = offset + leftEncoderValue / ticksPerRev * 360.0;
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
