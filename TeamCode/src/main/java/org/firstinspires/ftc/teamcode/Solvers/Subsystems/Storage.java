package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Globals.Robot;

import java.util.function.BooleanSupplier;

public class Storage extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private DigitalChannel transfer;
    public static boolean full = false;
    public double flapDown = 0.0;
    public double flapUp = 0.0;
    private boolean intaking = false;
    private boolean shooting = false; // Added to track shooting state

    BooleanSupplier notIntaking = () -> !intaking;

    public InstantCommand Intake = new InstantCommand(() -> {
        if(!full)
            robot.transferMotor.set(-1);
        robot.intakeMotor.set(-1);
        intaking = true;
    });

    public InstantCommand Outake = new InstantCommand(() -> {
        robot.transferMotor.set(0);
        robot.intakeMotor.set(0);
        intaking = false;
        shooting = false; // Reset shooting state when stopping
    });

    public SequentialCommandGroup SingularShot = new SequentialCommandGroup(
            new InstantCommand(() -> {
                shooting = true; // Ignore transfer state while shooting
                full = false;
                robot.intakeMotor.set(-1);
                robot.transferMotor.set(-1);
            }),
            new WaitCommand(500),
            Outake
    );

    public SequentialCommandGroup ContinousShot = new SequentialCommandGroup(
            new InstantCommand(() -> {
                shooting = true; // Ignore transfer state while shooting
                full = false;
                robot.intakeMotor.set(-1);
                robot.transferMotor.set(-1);
            }),
            new WaitUntilCommand(notIntaking),
            Outake
    );

    public SequentialCommandGroup Shoot() {
        if(intaking)
            return ContinousShot;
        return SingularShot;
    }

    public InstantCommand FlapDown = new InstantCommand(() -> {
        robot.flap.set(flapDown);
    });

    public InstantCommand FlapUp = new InstantCommand(() -> {
        robot.flap.set(flapUp);
    });
    public Storage(){
        transfer = robot.breakBeam1;
    }
    @Override
    public void periodic() {
        // Only update 'full' from the break beam if we are NOT currently shooting
        if (!shooting && transfer.getState()) {
            full = true;
        }
    }
}