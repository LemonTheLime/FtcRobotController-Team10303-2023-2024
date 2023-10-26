package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//runs the servos of the claw attachment
public class ClawControl{

    //FIELDS
    //Hardware map
    private HardwareMap hardwareMap = null;
    //Declare servos
    private Servo clawServo = null;
    private Servo pitchServo = null; //controls the pitch of the claw on the arm
    //Claw constants
    private boolean status;

    //CONSTRUCTOR
    /* Opmodes will create an instance of ClawControl class to run the claw servo
     * takes in the hardwaremap
     */
    public ClawControl(HardwareMap hwMap) {
        hardwareMap = hwMap;
        init();
    }

    //initialize servo
    private void init() {
        status = false;
        //get servo from id
        clawServo = hardwareMap.get(Servo.class, "claw");
        pitchServo = hardwareMap.get(Servo.class, "pitch");
    }

    //open the servo
    public void open() {
        clawServo.setPosition(1);
    }

    //close the servo
    public void close() {
        clawServo.setPosition(0);
    }

    //change the servo position
    public void change() {
        status = !status;
        if(status) {
            open();
        } else {
            close();
        }
    }

}
