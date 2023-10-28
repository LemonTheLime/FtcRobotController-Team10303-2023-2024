package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private boolean open;

    //CONSTRUCTOR
    /* Opmodes will create an instance of ClawControl class to run the claw servo
     * takes in the hardwaremap
     */
    public ClawControl(HardwareMap hwMap) {
        status = false;
        hardwareMap = hwMap;
        initHardware();
    }

    //initialize servo
    private void initHardware() {
        //get servo from id
        clawServo = hardwareMap.get(Servo.class, "claw");
        pitchServo = hardwareMap.get(Servo.class, "pitch");
    }

    //telemetry
    public void telemetryOutput(Telemetry t) {
        //telemetry
        t.addLine("ClawControl: ");
        t.addData("status", status);
        t.addData("open", open);
        t.addLine();
    }

    //initializes the claw control mechanism
    public void init() {
        status = true;
    }

    //open the servo
    private void open() {
        clawServo.setPosition(1);
    }

    //close the servo
    private void close() {
        clawServo.setPosition(0);
    }

    //change the servo position
    public void change() {
        if(status) {
            open = !open;
            if (open) {
                open();
            } else {
                close();
            }
        }
    }

}
