package org.firstinspires.ftc.teamcode.Solvers.CommandBase;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public abstract class Command implements BCommand{
    public Scheduler scheduler = Scheduler.getInstance();
    public boolean scheduled = false;
    public boolean finished = false;
    @Override
    public void schedule(){
        scheduler.schedule(this);
    }
    @Override
    public boolean isFinished(){
        return finished;
    }

    public abstract void end();
}

