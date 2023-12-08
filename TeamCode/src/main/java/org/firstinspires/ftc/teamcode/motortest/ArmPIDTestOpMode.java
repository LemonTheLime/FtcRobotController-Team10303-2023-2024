package org.firstinspires.ftc.teamcode.motortest;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ArmPIDTestOpMode
 * Testing for new pid control of arm
 */
@Disabled
@Config
@TeleOp(name = "ArmPIDTestOpMode", group = "Testing")
public class ArmPIDTestOpMode extends OpMode {

    //Proportional Integral Derivative Controller
    //Rev through bore encoder has 8192 encoder counts (2^13)
    public static double Kp = 0.00020926339;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double a = 0.8;
    public static double maxIntegralSum = 10;
    private int reference = 30; //ticks
    private int lastReference = 0;
    private double integralSum = 0;
    private int lastError = 0;
    private double previousFilterEstimate = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean setPointIsNotReached = true;

    //telemetry
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashTelemetry = null;



    //motor
    private DcMotorEx armMotor = null;
    private DcMotorEx armEncoder = null;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "rightArm");
        armEncoder = hardwareMap.get(DcMotorEx.class, "rightArm");

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //telemetry
        dashTelemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        // obtain the encoder position
        int encoderPosition = armMotor.getCurrentPosition();
        // calculate the error
        int error = reference - encoderPosition;

        int errorChange = (error - lastError);

        // filter out hight frequency noise to increase derivative performance
        double currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        double derivative = currentFilterEstimate / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());


        // max out integral sum
        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon setpoint changes
        if (reference != lastReference) {
            integralSum = 0;
        }

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        armMotor.setPower(out);

        lastError = error;

        lastReference = reference;

        // reset the timer for next time
        timer.reset();

        //telemetry
        telemetryOutput();
    }

    //telemetry function
    private void telemetryOutput() {
        dashTelemetry.addData("reference", reference);
        dashTelemetry.addData("integralSum", integralSum);
        dashTelemetry.addData("lastError", lastError);
        dashTelemetry.addData("previousFilterEstimate", previousFilterEstimate);

        dashTelemetry.update();
    }
}
