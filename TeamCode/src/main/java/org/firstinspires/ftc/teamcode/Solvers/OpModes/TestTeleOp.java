package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

@Config
public class TestTeleOp extends OpMode {
    Launcher shooter = new Launcher();
    public static int speed = 0;
    @Override
    public void init() {
        shooter.startShooter(5.0);

        telemetry.addData("ShooterSpeed", shooter.Speed);
        telemetry.addData("ShooterNear", shooter.Near);
    }

    @Override
    public void loop() {
        shooter.updateShooter();
        shooter.startShooter(speed);
        telemetry.addData("ShooterSpeed", shooter.Speed);
        telemetry.addData("ShooterNear", shooter.Near);
    }
}
