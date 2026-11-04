
package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;

@Configurable
public class Storage extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    private final DigitalChannel transferBeam;

    public static boolean full = false;

    public static double flapDown = 0.0;
    public static double flapUp = 0.0;

    private boolean intaking = false;
    private boolean shooting = false;

    // Intake
    public final InstantCommand Intake = new InstantCommand(() -> {
        robot.intakeMotor.set(-1);
        intaking = true;
    });

    public final InstantCommand Outake = new InstantCommand(() -> {
        robot.intakeMotor.set(0);
        intaking = false;

        if (!shooting) {
            robot.transferMotor.set(0);
        }
    });

    // Transfer
    public final InstantCommand IntakeT = new InstantCommand(() -> {
        robot.transferMotor.set(-1);
        full = false;
    });

    public final InstantCommand OutakeT = new InstantCommand(() -> {
        if (!intaking && !shooting) {
            robot.transferMotor.set(0);
        }
    });

    // Shooting
    public final InstantCommand SingularShot = new InstantCommand(() -> {
        shooting = true;
        full = false;

        robot.intakeMotor.set(-1);
        robot.transferMotor.set(-1);
    });

    public InstantCommand shootCommand() {
        return new InstantCommand(() -> {
            shooting = true;
            full = false;

            // Run both motors simultaneously
            robot.intakeMotor.set(-1);
            robot.transferMotor.set(-1);
        });
    }

    public InstantCommand stopShootingCommand() {
        return new InstantCommand(() -> {
            shooting = false;
            robot.intakeMotor.set(0);
            robot.transferMotor.set(0);
            intaking = false;
        });
    }


    // Flap
    public final InstantCommand FlapDown = new InstantCommand(() ->
            robot.flap.set(flapDown)
    );

    public final InstantCommand FlapUp = new InstantCommand(() ->
            robot.flap.set(flapUp)
    );

    public Storage() {
        transferBeam = robot.breakBeam1;
    }

    @Override
    public void periodic() {
        if (!shooting && !transferBeam.getState()) {
            full = true;
        }
    }
}
