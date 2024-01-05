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

    //drivetrain
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;
    double speedRatio = 0.75; // 3/4
    double rotationRatio = 0.67; // 2/3
    //arm
    private ArmControl Arm = null;
    private double armRotation;
    private final double ARM_SPEED = 12.0;
    //claw
    private ClawControl Claw = null;
    private double pitchRotation;
    private final double PITCH_SPEED = 0.02;
    private boolean changeLeftClaw = false;
    private boolean changeRightClaw = false;
    private boolean changeAllClaw = false;
    //launcher
    private LauncherControl Launcher = null;
    private boolean runLauncher = false;
    //gamepads
    private String lastKeyPressed1 = "none";
    private String lastKeyPressed2 = "none";

    //create hardware map
    public void init() {
        //drivetrain hardware and referse left wheels
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
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

        //Arm.updatePIDF();
    }

    //TeleOp init the motors
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
        Arm.rotate(armRotation * ARM_SPEED);
        Arm.update(); //arm will run depending on current arm state
        //run claws
        Claw.rotate(pitchRotation * PITCH_SPEED);
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
        runLauncher = gamepad1.b && gamepad2.b;

        //SINGLE PRESS BUTTONS
        //decrease drivetrain speed and rotation ratios
        if(gamepad1.left_bumper) {
            if(lastKeyPressed1.equals("none")) {
                lastKeyPressed1 = "left_bumper";
                speedRatio -= 0.15;
                rotationRatio -= 0.13;
                if(speedRatio <= 0) {
                    speedRatio += 0.15;
                }
                if(rotationRatio <= 0) {
                    rotationRatio += 0.13;
                }
            }
        }

        //increase drivetrain speed and rotation ratios
        if(gamepad1.right_bumper) {
            if(lastKeyPressed1.equals("none")) {
                lastKeyPressed1 = "right_bumper";
                speedRatio += 0.15;
                rotationRatio += 0.13;
                if(speedRatio > 1.5) {
                    speedRatio -= 0.15;
                }
                if(rotationRatio > 1.3) {
                    rotationRatio -= 0.13;
                }
            }
        }

        //arm prepares to drop pixel
        if(gamepad2.a) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "a";
                Claw.deliver();
                Arm.deliver();
            }
        }
        //arm to ground state
        if(gamepad2.x) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "x";
                Claw.ground();
                Arm.ground();
            }
        }
        //arm to reset state
        if(gamepad2.y) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "y";
                Arm.reset();
                Claw.reset();
            }
        }
        //reset arm encoders
        if(gamepad2.dpad_up) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "dpad_up";
                Arm.resetEncoder();
            }
        }
        //change left claw
        if(gamepad2.dpad_left) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "dpad_left";
                changeLeftClaw = true;
            }
        }
        //change right claw
        if(gamepad2.dpad_right) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "dpad_right";
                changeRightClaw = true;
            }
        }
        //either open or close both claws
        if(gamepad2.dpad_down) {
            if(lastKeyPressed2.equals("none")) {
                lastKeyPressed2 = "dpad_down";
                changeAllClaw = true;
            }
        }


        //gamepad1 single button press reset
        if(!gamepad2.left_bumper && !gamepad1.right_bumper) {
            lastKeyPressed1 = "none";
        }

        //gamepad2 single button press reset
        if(!gamepad2.a && !gamepad2.x && !gamepad2.y && !gamepad2.dpad_left
                && !gamepad2.dpad_right && !gamepad2.dpad_up
                && !gamepad2.dpad_down && !gamepad2.left_bumper) {
            lastKeyPressed2 = "none";
        }

        //testing update pidf
        /*
        if(gamepad2.left_bumper) {
            Arm.updatePIDF();
            telemetry.addLine("Something");
        }

         */
    }

    //telemetry
    private void telemetryOutput() {
        Arm.telemetryOutput();
        Claw.telemetryOutput();
        Launcher.telemetryOutput();
        telemetry.update();
    }
}
