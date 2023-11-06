package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.attachments.ArmControl;
import org.firstinspires.ftc.teamcode.attachments.ClawControl;
import org.firstinspires.ftc.teamcode.attachments.LauncherControl;

@TeleOp(name="TeleOpMode")
public class TeleOpMode extends OpMode {

    //FIELDS
    //drivetrain
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;
    double speedRatio = 1.0;
    //arm
    private ArmControl Arm = null;
    private double armRotation;
    private double armSpeed = 3.0;
    //claw
    private ClawControl Claw = null;
    private double pitchRotation;
    private double pitchSpeed = 0.02;
    private boolean changeClaw = false;
    //launcher
    private LauncherControl Launcher = null;
    private boolean runLauncher = false;
    //gamepads
    private String lastKeyPressed = "";
    private boolean buttonPressed = false;


    @Override
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
        Arm.init();

        //Claw mechanism
        Claw = new ClawControl(hardwareMap, telemetry);
        Claw.init();

        //Launcher control
        Launcher = new LauncherControl(hardwareMap, telemetry);
        Launcher.init();
    }

    @Override
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
        //run claw
        Claw.rotate(pitchRotation * pitchSpeed);
        if(changeClaw) {
            changeClaw = false;
            Claw.changeClaw();
        }
        //run launcher
        if(runLauncher) {
            Launcher.run();
        } else {
            Launcher.stop();
        }
    }

    //read the gamepad inputs
    private void readPlayerInputs() {
        //gamepad1 - drivetrain
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
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
        leftFrontPower *= speedRatio;
        rightFrontPower *= speedRatio;
        leftRearPower *= speedRatio;
        leftRearPower *= speedRatio;

        //gamepad2 - attachments
        armRotation = gamepad2.left_stick_y; //moving stick up will flip the arm out
        pitchRotation = -gamepad2.right_stick_y; //moving stick up will flip the claw out

        //open or close claw - click "a" once
        if(gamepad2.a) {
            if(lastKeyPressed.equals("none")) {
                lastKeyPressed = "a";
                changeClaw = true;
            }
        }
        //gamepad2 single button press reset
        if(!gamepad2.a) {
            lastKeyPressed = "none";
        }

        //hold b for launcher
        if(gamepad2.b) {
            runLauncher = true;
        } else {
            runLauncher = false;
        }

        //press x to set pitch servo to ground
        if(gamepad2.x) {
            Claw.rotateToGround();
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
