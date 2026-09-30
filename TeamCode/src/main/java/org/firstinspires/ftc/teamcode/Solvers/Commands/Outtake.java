package org.firstinspires.ftc.teamcode.Solvers.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;

public class Outtake extends CommandBase {
    private final Robot robot = Robot.getInstance();
    boolean initialized = false;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void initialize(){
        initialized = true;
    }

    public void reset(){
        initialized = true;
        robot.launcher.setFlap(false);
    }

    public void execute(){
        if (initialized) {
            robot.launcher.setFlap(true);
        }
        else{
            reset();
        }
    }

    public boolean isFinished(){
        return initialized;
    }

    public void end(){
        initialized = false;
        robot.launcher.setFlap(false);
    }
}
