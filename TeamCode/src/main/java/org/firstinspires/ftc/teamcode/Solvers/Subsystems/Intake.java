package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import java.util.function.BooleanSupplier;

import kotlin.jvm.JvmField;

@Config
public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    ElapsedTime elapsedTime2 = new ElapsedTime();

    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime elapsedTime3 = new ElapsedTime();
    Boolean checkedForCurrent = false;
    public boolean runningAuto = false;
    public static double CurrentThreshold = 6.0;
    public static double tolerance = 50;
    public static double intakeUp = 0;
    public static double intakeDown = 0.2;
    public BooleanSupplier supplier = () -> Storage.full;
    Double current = 0.0;
    boolean lastResponse = true;
    boolean overloaded = false;
    public boolean full = false;
    public boolean startClear = false;
    public ElapsedTime elapsedTime4 = new ElapsedTime();
    static double intakePos;

    public Intake() {
        elapsedTime.startTime();
        elapsedTime2.startTime();
        intakeDown();

    }
    public void outake(){
        robot.transferMotor.set(1);
        robot.intakeMotor.set(1);
    }

    public void intake(Boolean onoff) {
        if (Launcher.isFlapOpen) {
            if(robot.launcher.flapOpen) {
                robot.transferMotor.set(onoff ? -1 : 0);
                robot.intakeMotor.set(onoff ? -1 : 0);
            }else {
                robot.transferMotor.set(0);
                robot.intakeMotor.set(0);
            }
        }
        robot.transferMotor.set((elapsedTime4.milliseconds()>50 || Launcher.isFlapOpen) && onoff ? -0.8 : 0);

        robot.intakeMotor.set(onoff ? Storage.full ? 0:-1 : 0);


        if (full && Launcher.isFlapOpen && onoff) {
            if (!startClear) {
                elapsedTime3.startTime();
                elapsedTime3.reset();
            }
            startClear = true;
        }

    }
    public void intake(Boolean onoff,boolean s) {
        if (Launcher.isFlapOpen) {
            if(robot.launcher.flapOpen) {
                robot.transferMotor.set(onoff ? -1 : 0);
                robot.intakeMotor.set(onoff ? -1 : 0);
            }else {
                robot.transferMotor.set(0);
                robot.intakeMotor.set(0);
            }
        }
        robot.transferMotor.set((elapsedTime4.milliseconds()>50 || Launcher.isFlapOpen) && onoff ? -0.7 : 0);

        robot.intakeMotor.set(onoff ? Storage.full ? 0:-0.7 : 0);


        if (full && Launcher.isFlapOpen && onoff) {
            if (!startClear) {
                elapsedTime3.startTime();
                elapsedTime3.reset();
            }
            startClear = true;
        }

    }
    public void intake() {
        boolean onoff = true;
        if (Launcher.isFlapOpen) {
            if(robot.launcher.flapOpen) {
                robot.transferMotor.set(-1);
                robot.intakeMotor.set(-1);
            }else {
                robot.transferMotor.set(0);
                robot.intakeMotor.set(0);
            }
        }
        robot.transferMotor.set(elapsedTime4.milliseconds() > 50 || Launcher.isFlapOpen ? -1 : 0);

        robot.intakeMotor.set(Storage.full ? 0:-1);


        if (full && Launcher.isFlapOpen) {
            if (!startClear) {
                elapsedTime3.startTime();
                elapsedTime3.reset();
            }
            startClear = true;
        }

    }

    @Override
    public void periodic() {
        update();
        int i = 0;
        if(robot.storage.list[1])
            i++;
        if(robot.storage.list[2])
            i++;
        if(i>1)
            elapsedTime4.reset();
        //if(robot.storage.)
    }
    public static void intakeUp(){
        intakePos = intakeUp;

    }
    public static void intakeDown(){
        intakePos = intakeDown;

    }
    public static void intakeDownDown(){
        intakePos = intakeDown+0.1;
    }
    void update(){
        if(runningAuto)
            robot.intakeServo.set(intakeDown);
        else
            robot.intakeServo.set(intakePos);
        if (Launcher.isFlapOpen) {
            overloaded = false;
        }
        if (startClear) {
            if (elapsedTime3.milliseconds() > 100) {
                startClear = false;
                full = false;
            }
        }
    }


    boolean canPushTransfer() {
        if (Launcher.isFlapOpen) {
            elapsedTime2.reset();
            return true;
        }
        if (overloaded) {
            return false;
        }
        if (!checkedForCurrent || elapsedTime.milliseconds() > 100) {
            current = robot.transferMotor.getCurrent(CurrentUnit.AMPS);
            if (!checkedForCurrent) {
                elapsedTime.reset();
            }
        }
        if (current > CurrentThreshold) {
            elapsedTime2.reset();
            overloaded = true;
            return false;
        }

        if (lastResponse || elapsedTime2.milliseconds() > 200) {
            elapsedTime2.reset();
            overloaded = false;
            return true;
        }

        elapsedTime2.reset();
        overloaded = true;
        return false;
    }


}

