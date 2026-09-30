package org.firstinspires.ftc.teamcode.Solvers.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Globals.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class Scheduler{
    private final Robot robot = Robot.getInstance();;
    private static final ElapsedTime elapsedTime = new ElapsedTime();
    private static Scheduler instance = new Scheduler();
    public ArrayList<Command> scheduledCommands = new ArrayList<>();;
    private final Map<Subsystem, Double> subsystems= new HashMap<>();

    public void schedule(Command command){
        if(command.scheduled) return;
        command.scheduled = true;
        command.initialize();
        scheduledCommands.add(command);
    }
    public void registerSubsystem(Subsystem... subsystems_) {
        for (Subsystem subsystem : subsystems_) {
            subsystems.put(subsystem,0.0);
        }
    }


    public static Scheduler getInstance() {
        if (instance == null) {
            instance = new Scheduler();
        }
        return instance;
    }
    public void periodic() {
        for(Subsystem subsystem: subsystems.keySet()){
            double start = elapsedTime.milliseconds();
            subsystem.periodic();
            double time = start- elapsedTime.milliseconds();
            subsystems.put(subsystem, time);
        }
        Iterator<Command> iterator = scheduledCommands.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            if (!command.scheduled && command.finished) {
                command.end();
                iterator.remove();
            }else {
                if(command.isFinished()){
                    command.scheduled = false;
                    command.end();
                    iterator.remove();
                }else {
                    command.execute();
                }
            }
        }
    }
}
