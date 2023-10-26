package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//runs the motors of the arm attachment
public class ArmControl{

    //FIELDS
    //Hardware map
    private HardwareMap hardwareMap = null;
    //Declare motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    //Motor Constants:
    private int ticksPerRev;

    //CONSTRUCTOR
    /* Opmodes will create an instance of ArmControl class to run the arm motors
     * takes in the hardwaremap
     */
    public ArmControl(HardwareMap hwMap) {
        hardwareMap = hwMap;
        init();
    }

    //initialize motors
    private void init() {
        //get motors from ids
        leftMotor = hardwareMap.get(DcMotor.class, "leftArm");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        //reverse motors here if needed:


        //set motors to run with encoders to target positions
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



}
