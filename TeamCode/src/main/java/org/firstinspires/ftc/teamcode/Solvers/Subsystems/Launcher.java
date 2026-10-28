package org.firstinspires.ftc.teamcode.Solvers.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

public class Launcher extends SubsystemBase {
    public static PIDCoefficients testPid = new PIDCoefficients(0.01, 0, 0);
    public static BasicFeedforwardParameters timeUnit = new BasicFeedforwardParameters(0.00045, 0,  0.32);
    ControlSystem spinner = ControlSystem.builder().velPid(testPid).basicFF(timeUnit).build();
    private final Robot robot = Robot.getInstance();

    public boolean Near = false;
    public double Speed = -1500;


    private void setGoal(double speed) {
        spinner.setGoal(new KineticState(0,Math.min(speed, Constants.LAUNCHER_MAX_VELOCITY)));
    }
    private void update() {
        double output = spinner.calculate(new KineticState(0, robot.launchEncoder.getCorrectedVelocity()));
        robot.launchMotor.set(output);
    }

    public void startShooter(double speed) {
        //robot.hoodServo.set(0.7);
        this.Speed = speed;
        setGoal(speed);
    }

    public void updateShooter() {
        update();

        if (Math.abs(robot.launchMotor.getCorrectedVelocity() - Math.min(this.Speed, Constants.LAUNCHER_MAX_VELOCITY)) <= 0.25) {
            System.out.println("NEARRRRRR");
            this.Near = true;
        }
        else {
            this.Near = false;
        }
    }
}
