package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

@Config
@TeleOp
public class TestTeleOp extends OpMode {

    private final Robot robot = Robot.getInstance();
    private final Launcher shooter = new Launcher();

    public static int speed = -1500;
    public static double DRIVE_SPEED = 1.0;

    @Override
    public void init() {
        robot.init(hardwareMap);

        shooter.startShooter(1000);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // ========================
        // Mecanum Drive
        // ========================

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Mecanum wheel powers
        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        // Normalize powers to prevent exceeding 1.0
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1.0
        );

        frontLeftPower /= denominator;
        backLeftPower /= denominator;
        frontRightPower /= denominator;
        backRightPower /= denominator;

        // Apply drive speed multiplier
        robot.frontLeft.set(frontLeftPower * DRIVE_SPEED);
        robot.backLeft.set(backLeftPower * DRIVE_SPEED);
        robot.frontRight.set(frontRightPower * DRIVE_SPEED);
        robot.backRight.set(backRightPower * DRIVE_SPEED);

        // ========================
        // Shooter
        // ========================

        shooter.updateShooter();
        shooter.startShooter(speed);

        // ========================
        // Intake
        // ========================

        robot.intakeMotor.set(-gamepad1.right_trigger);
        robot.transferMotor.set(-gamepad1.left_trigger);

        // ========================
        // Telemetry
        // ========================

        telemetry.addData("ShooterSpeed", shooter.Speed);
        telemetry.addData("ShooterNear", shooter.Near);

        telemetry.addData("Front Left", frontLeftPower);
        telemetry.addData("Front Right", frontRightPower);
        telemetry.addData("Back Left", backLeftPower);
        telemetry.addData("Back Right", backRightPower);

        telemetry.update();
    }
}