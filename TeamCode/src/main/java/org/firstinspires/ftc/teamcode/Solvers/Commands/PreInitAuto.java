package org.firstinspires.ftc.teamcode.Solvers.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

public class PreInitAuto  extends CommandBase {
    private final Robot robot;
    boolean initialized = false;
    public PreInitAuto(){
        robot = Robot.getInstance();
    }

    @Override
    public void initialize() {
        robot.launcher.doFlywheel = true;
        robot.launcher.setFlap(true);

    }
    @Override
    public void execute() {
        robot.launcher.doFlywheel = true;
        robot.launcher.setFlap(true);
        initialized = true;
    }
    @Override
    public boolean isFinished() {

        return initialized;
    }
}
