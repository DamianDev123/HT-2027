package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@TeleOp(name = "testOp4")
@Config
public class testOp4 extends OpMode {

    public static double power1 = 0.0;
    public static double power2 = 0.0;
    public static double power3 = 0.0;
    public static double power4 = 0.0;
    public static double power5 = 0.0;
    public static double power6 = 0.0;

    private ServoEx s1, s2, s3, s4, s5, s6;

    @Override
    public void init() {
        s1 = new ServoEx(hardwareMap, "s1").setCachingTolerance(0.001);
        s2 = new ServoEx(hardwareMap, "s2").setCachingTolerance(0.001);
        s3 = new ServoEx(hardwareMap, "s3").setCachingTolerance(0.001);
        s4 = new ServoEx(hardwareMap, "s4").setCachingTolerance(0.001);
        s5 = new ServoEx(hardwareMap, "s5").setCachingTolerance(0.001);
        s6 = new ServoEx(hardwareMap, "s6").setCachingTolerance(0.001);
    }

    @Override
    public void loop() {
        s1.set(power1);
        s2.set(power2);
        s3.set(power3);
        s4.set(power4);
        s5.set(power5);
        s6.set(power6);

        telemetry.addData("s1", power1);
        telemetry.addData("s2", power2);
        telemetry.addData("s3", power3);
        telemetry.addData("s4", power4);
        telemetry.addData("s5", power5);
        telemetry.addData("s6", power6);
        telemetry.update();
    }
}