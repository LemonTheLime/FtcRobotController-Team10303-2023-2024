package org.firstinspires.ftc.teamcode.test.motortest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This OpMode uses the extended/enhanced
 * PIDF-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * deprecated Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */

@Disabled
@Autonomous(name="Concept: Change PIDF", group = "Concept")
public class ConceptChangePIDF extends LinearOpMode {

    // our DC motor
    DcMotorEx motorExLeft;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0.5;
    // These values are for illustration only; they must be set
    // and adjusted for each motor based on its planned usage.

    public void runOpMode() {
        // Get reference to DC motor.
        // Since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightArm");

        // wait for start command
        waitForStart();

        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfRTP = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidfRWE = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        //PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfNew);

        // Re-read coefficients and verify change.
        //PIDFCoefficients pidfModified = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // display info to user
        while(opModeIsActive()) {
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (RTP)", "%.04f, %.04f, %.04f, %.04f",
                    pidfRTP.p, pidfRTP.i, pidfRTP.d, pidfRTP.f);
            telemetry.addData("P,I,D,F (RWE)", "%.04f, %.04f, %.04f, %.04f",
                    pidfRWE.p, pidfRWE.i, pidfRWE.d, pidfRWE.f);
            //telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
            //        pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.update();
        }
    }
}