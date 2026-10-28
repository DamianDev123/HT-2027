package org.firstinspires.ftc.teamcode.Solvers.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

public class TestTeleOp extends OpMode {
    Launcher shooter = new Launcher();
    @Override
    public void init() {

        shooter.startShooter(5.0);
    }

    @Override
    public void loop() {
        shooter.updateShooter();
        telemetry.addData("ShooterSpeed", shooter.Speed);
    }
}
