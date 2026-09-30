package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Subsystem;

public class Storage extends Subsystem {
    public static boolean full = false;
    public boolean contains = false;
    public boolean setup = false;
    public boolean[] list = {true,true,true};
    public DigitalChannel stage1 = null;
    public DigitalChannel stage2 = null;
    public Robot robot = Robot.getInstance();

    public DigitalChannel stage3 = null;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    public boolean emptyF = false;
    public Storage() {
        timer.startTime();
        timer.reset();
        timer2.startTime();
        timer2.reset();
    }
    void update(){

        if(stage1==null) return;
        list[0] = !stage1.getState();
        list[1] = !stage2.getState();
        list[2] = !stage3.getState();
        if(list[0] && list[1] && list[2]){
            Intake.intakeUp();
        }else {
            timer2.reset();
        }
        if(full && !setup){
            setup = true;
            Intake.intakeUp();
        }
        if(timer2.milliseconds()>500)
            full = true;
        if(Launcher.isFlapOpen){
            full = false;
            setup = false;
        }
        contains = list[0] || list[1] || list[2];
        if(contains)
            timer.reset();
        emptyF = timer.milliseconds()>100;
    }
    public void resetFull(){
        full = list[0] && list[1] && list[2];
    }
    @Override
    public void periodic() {
        update();
    }
}
