package org.firstinspires.ftc.teamcode.motortest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* DashboardTest
 * Tests out the FTC Dashboard Config variables
 */

@Disabled
@Config
@TeleOp(name = "DashboardTest", group = "Testing")
public class DashboardTest extends OpMode {

    public static int num = 0;
    public static String field = "test123";

    @Override
    public void init() {
        telemetry.addLine("Start OpMode");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("num", num);
        telemetry.addData("field", field);
        telemetry.update();
    }
}
