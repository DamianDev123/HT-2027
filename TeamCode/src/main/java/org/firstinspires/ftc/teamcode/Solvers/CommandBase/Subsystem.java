package org.firstinspires.ftc.teamcode.Solvers.CommandBase;
public abstract class Subsystem {
    public Subsystem(){Scheduler.getInstance().registerSubsystem(this);}
    public void periodic(){

    }
}
