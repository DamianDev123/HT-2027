package org.firstinspires.ftc.teamcode.Solvers.Subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@Configurable
public class Turret extends SubsystemBase {

    public static PIDCoefficients turretPid =
            new PIDCoefficients(0.01, 0, 0);

    public static BasicFeedforwardParameters turretFF =
            new BasicFeedforwardParameters(0, 0, 0);

    public static ControlSystem turretController =
            ControlSystem.builder()
                    .posPid(turretPid)
                    .basicFF(turretFF)
                    .build();

    private final Robot robot = Robot.getInstance();

    // Gear ratios
    public static final double MOTOR_GEAR_RATIO = 5.2;
    public static final double EXTERNAL_GEAR_RATIO = 73.0 / 11.0;

    public static final double TICKS_PER_REV =
            28.0 * MOTOR_GEAR_RATIO * EXTERNAL_GEAR_RATIO;

    public static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    public static final double DEGREES_PER_TICK =
            1.0 / TICKS_PER_DEGREE;

    // Turret configuration
    public static double MIN_ANGLE = -50;
    public static double MAX_ANGLE = 50;
    public static double TOLERANCE = 2.0;

    // Alignment configuration
    public static boolean AUTO_ALIGN = true;
    public static double HIVE_OFFSET = 0;

    public double currentAngle = 0;
    public static double TargetAngle = 0;
    public boolean Near = false;

    // Normalize angle to -180 to 180
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // Convert field position into turret-relative angle
    public double getHiveAngle() {

        Pose robotPose = robot.getPose();
        Pose hivePose = robot.getHive();

        double deltaX = hivePose.x() - robotPose.x();
        double deltaY = hivePose.y() - robotPose.y();

        double fieldAngle = Math.toDegrees(
                Math.atan2(deltaY, deltaX)
        );

        double robotHeading = Math.toDegrees(robotPose.heading());
        double tt = fieldAngle - robotHeading + HIVE_OFFSET;
        if(Launcher.ServoPos>0.72)
            tt+=180;
        return -normalizeAngle(tt);
    }

    // Set turret target
    public void setTargetAngle(double angle) {

        TargetAngle = Math.max(
                MIN_ANGLE,
                Math.min(angle, MAX_ANGLE)
        );

        double targetTicks = TargetAngle * TICKS_PER_DEGREE;

        turretController.setGoal(
                new KineticState(targetTicks, 0)
        );
    }

    // Update turret PID
    private void update() {

        double currentPosition =
                robot.turretEncoder.getPosition();

        double output = turretController.calculate(
                new KineticState(currentPosition, 0)
        );

        robot.turretMotor.set(output);
    }

    // Update turret position
    public void updateTurret() {

        update();

        currentAngle =
                robot.turretEncoder.getPosition()
                        / TICKS_PER_DEGREE;

        Near = Math.abs(
                normalizeAngle(TargetAngle - currentAngle)
        ) <= TOLERANCE;
    }

    // Enable automatic hive alignment
    public void alignToHive() {
        AUTO_ALIGN = true;
    }

    // Disable automatic hive alignment
    public void disableAutoAlign() {
        AUTO_ALIGN = false;
    }

    public void stop() {
        robot.turretMotor.set(0);
    }

    @Override
    public void periodic() {

        if (AUTO_ALIGN) {
            setTargetAngle(getHiveAngle());
        }
        updateTurret();
    }
}