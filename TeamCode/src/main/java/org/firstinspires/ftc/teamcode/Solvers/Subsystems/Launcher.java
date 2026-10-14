package org.firstinspires.ftc.teamcode.Solvers.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
@Config
public class Launcher extends SubsystemBase {
    public static PIDCoefficients testPid = new PIDCoefficients(0.1, 0, 0);
    public BasicFeedforwardParameters timeUnit = new BasicFeedforwardParameters(0.00045, 0,  0.32);
    ControlSystem spinner = ControlSystem.builder().velPid(testPid).basicFF(timeUnit).build();
    public void startLauncher() {
        spin();
    }
    boolean ready = false;
    private final Robot robottt = Robot.getInstance();
    public void spin() {
        spinner.setGoal(new KineticState(0,Math.min(1000.0, Constants.LAUNCHER_MAX_VELOCITY)));
        ready = true;
    }

    public void periodic() {
        if (ready) {
            double output = spinner.calculate(new KineticState(0, robottt.launchEncoder.getCorrectedVelocity()));
            robottt.launchMotors.set(output);
        } else {
            spin();
        }
    }
}
