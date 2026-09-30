package org.firstinspires.ftc.teamcode.Solvers.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;

import java.util.function.BooleanSupplier;

public class WaitFull extends CommandBase {

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void initialize() {
        time.startTime();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        return Storage.full || time.milliseconds()>1000;
    }

    @Override
    public void end(boolean interrupted) {
        time.reset();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}