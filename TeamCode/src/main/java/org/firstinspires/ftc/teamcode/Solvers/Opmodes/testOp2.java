package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@TeleOp
public class testOp2 extends OpMode {
    public DigitalChannel stage1;
    public DigitalChannel stage2;

    public DigitalChannel stage3;
    public DigitalChannel stage4;
    public DigitalChannel stage5;

    public DigitalChannel stage6;
    public static double targetDegrees = 0.0;

    @Override
    public void init() {
        stage1 = hardwareMap.get(DigitalChannel.class, "d1");
        stage2 = hardwareMap.get(DigitalChannel.class, "d2");
        stage3 = hardwareMap.get(DigitalChannel.class, "d3");
        stage4 = hardwareMap.get(DigitalChannel.class, "d4");
        stage5 = hardwareMap.get(DigitalChannel.class, "d5");
        stage6 = hardwareMap.get(DigitalChannel.class, "d6");

    }

    @Override
    public void loop() {


        // Telemetry for debugging
        telemetry.addData("Target Angle1", stage1.getState());
        telemetry.addData("Target Angle2", stage2.getState());
        telemetry.addData("Target Angle3", stage3.getState());
        telemetry.addData("Target Angle4", stage4.getState());
        telemetry.addData("Target Angle5", stage5.getState());
        telemetry.addData("Target Angle6", stage6.getState());
        telemetry.update();
    }
}