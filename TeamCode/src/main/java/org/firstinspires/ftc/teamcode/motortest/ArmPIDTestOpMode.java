package org.firstinspires.ftc.teamcode.motortest;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmPIDTestOpMode extends OpMode {

    //Proportional Integral Derivative Controller
    //Rev through bore encoder has 8192 encoder counts (2^13)
    private double Kp = 0.00020926339;
    private double Ki = 0;
    private double Kd = 0;
    private int reference = 30; //ticks
    private double integralSum = 0;
    private int lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private boolean setPointIsNotReached = true;

    //motor
    private DcMotorEx armMotor = null;
    private DcMotorEx armEncoder = null;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "rightArm");
        armEncoder = hardwareMap.get(DcMotorEx.class, "rightArm");
    }

    @Override
    public void loop() {
        while (setPointIsNotReached) {
            // obtain the encoder position
            int encoderPosition = armEncoder.getCurrentPosition();
            // calculate the error
            int error = reference - encoderPosition;

            // rate of change of the error
            double derivative = ((double)(error - lastError)) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            armMotor.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }
    }
}
