package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

@TeleOp
public class TestTeleOp extends OpMode {

    private final Robot robot = Robot.getInstance();
    private final Launcher shooter = new Launcher();

    public static int speed = -1500;
    public static double DRIVE_SPEED = 1.0;
    GamepadEx gm1;



    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);

        gm1 = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {

        telemetry.addData("Front Left", robot.breakBeam1.getState());
        telemetry.update();
        robot.updateLoop();
    }
}