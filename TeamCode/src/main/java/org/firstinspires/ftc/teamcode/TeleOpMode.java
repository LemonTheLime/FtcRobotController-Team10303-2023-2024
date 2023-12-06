package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.attachments.LauncherControl;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends OpMode {

    //FIELDS
    //drivetrain
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;
    double speedRatio = 0.75;
    double rotationRatio = 0.67;
    //arm
    private ArmControl Arm = null;
    private double armRotation;
    private double armSpeed = 1.5;
    //claw
    private ClawControl Claw = null;
    private double pitchRotation;
    private double pitchSpeed = 0.02;
    private boolean changeLeftClaw = false;
    private boolean changeRightClaw = false;
    private boolean changeAllClaw = false;
    //launcher
    private LauncherControl Launcher = null;
    private boolean runLauncher = false;
    //gamepads
    private String lastKeyPressed = "";


    public void init() {
        //init drivetrain hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        //reverse left wheels
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        //init motor power
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        rightRearPower = 0;

        //Arm mechanism
        Arm = new ArmControl(hardwareMap, telemetry);

        //Claw mechanism
        Claw = new ClawControl(hardwareMap, telemetry);

        //Launcher control
        Launcher = new LauncherControl(hardwareMap, telemetry);

        Arm.updatePIDF();
    }

    //TeleOp init
    public void start() {
        Arm.init();
        Claw.init();
        Launcher.init();
    }

    //Teleop directly controls the drivetrain because autonomous is separate
    public void loop() {
        readPlayerInputs();
        runDrivetrain();
        runAttachments();
        telemetryOutput();
    }

    //runs motors for the drivetrain
    private void runDrivetrain() {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    //run the attachments
    private void runAttachments() {
        //run arm
        Arm.rotate(armRotation * armSpeed);
        //run claws
        Claw.rotate(pitchRotation * pitchSpeed);
        if(changeLeftClaw) {
            changeLeftClaw = false;
            Claw.changeLeft();
        }
        if(changeRightClaw) {
            changeRightClaw = false;
            Claw.changeRight();
        }
        if(changeAllClaw) {
            //change both claws
            changeAllClaw = false;
            Claw.changeBothClaw();
        }
        //run launcher
        if(runLauncher) {
            Launcher.open();
        }
    }

    //read the gamepad inputs
    private void readPlayerInputs() {
        //gamepad1 - drivetrain
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y * speedRatio;  // Note: pushing stick forward gives negative value
        double lateral = -gamepad1.left_stick_x * speedRatio;
        double yaw     = gamepad1.right_stick_x * rotationRatio;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower  = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftRearPower   = axial - lateral + yaw;
        rightRearPower  = axial + lateral - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftRearPower   /= max;
            rightRearPower  /= max;
        }

        //gamepad2 - attachments
        armRotation = -gamepad2.left_stick_y; //moving stick up will flip the arm out
        pitchRotation = -gamepad2.right_stick_y; //moving stick up will flip the claw out

        //hold b for launcher (both needed)
        if(gamepad1.b && gamepad2.b) {
            runLauncher = true;
        } else {
            runLauncher = false;
        }

        //SINGLE PRESS BUTTONS
        //arm prepares to drop pixel
        if(gamepad2.a) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "a";
                Arm.deliver();
            }
        }
        //arm to ground state
        if(gamepad2.x) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "x";
                Claw.ground();
                Arm.ground();
            }
        }
        //arm to reset state
        if(gamepad2.y) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "y";
                Arm.reset();
                Claw.reset();
            }
        }
        //reset arm encoders
        if(gamepad2.dpad_up) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "dpad_up";
                Arm.resetEncoder();
            }
        }
        //change left claw
        if(gamepad2.dpad_left) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "dpad_left";
                changeLeftClaw = true;
            }
        }
        //change right claw
        if(gamepad2.dpad_right) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "dpad_right";
                changeRightClaw = true;
            }
        }
        //either open or close both claws
        if(gamepad2.dpad_down) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "dpad_down";
                changeAllClaw = true;
            }
        }

        //test update pid
        if(gamepad2.left_bumper) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "left_bumper";
                Arm.updatePIDF();
            }
        }


        //gamepad2 single button press reset
        if(!gamepad2.a && !gamepad2.x && !gamepad2.y && !gamepad2.dpad_left
                && !gamepad2.dpad_right && !gamepad2.dpad_up
                && !gamepad2.dpad_down && !gamepad2.left_bumper) {
            lastKeyPressed = "none";
        }
    }

    //telemetry
    private void telemetryOutput() {
        Arm.telemetryOutput();
        Claw.telemetryOutput();
        Launcher.telemetryOutput();
        telemetry.update();
    }
}
