
package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@Configurable
public class sTurret extends SubsystemBase {

    public static PIDCoefficients turretPid =
            new PIDCoefficients(0.01, 0, 0);

    public static double velFF = 0;

    public static BasicFeedforwardParameters turretFF =
            new BasicFeedforwardParameters(0, 0, 0);

    public static ControlSystem turretController =
            ControlSystem.builder()
                    .posPid(turretPid)
                    .basicFF(turretFF)
                    .build();

    private final Robot robot = Robot.getInstance();

    // ==========================================
    // GEAR RATIOS
    // ==========================================

    public static final double MOTOR_GEAR_RATIO = 5.2;
    public static final double EXTERNAL_GEAR_RATIO = 73.0 / 11.0;

    public static final double TICKS_PER_REV =
            28.0 * MOTOR_GEAR_RATIO * EXTERNAL_GEAR_RATIO;

    public static final double TICKS_PER_DEGREE =
            TICKS_PER_REV / 360.0;

    // ==========================================
    // MECHANICAL LIMITS
    // ==========================================

    public static double MIN_ANGLE = -234;
    public static double MAX_ANGLE = 418;

    public static double TOLERANCE = 2.0;

    // ==========================================
    // AUTOMATIC ALIGNMENT
    // ==========================================

    public static boolean AUTO_ALIGN = true;

    // Mechanical calibration correction.
    public static double HIVE_OFFSET = -9;

    // ==========================================
    // TURRET PHYSICAL OFFSET
    // ==========================================

    // Distance from robot localization origin
    // to turret pivot, in inches.
    //
    // Positive X = robot forward.
    // Positive Y = robot left.

    public static double TURRET_OFFSET_X = 0;
    public static double TURRET_OFFSET_Y = 0;

    // ==========================================
    // TURRET STATE
    // ==========================================

    public double currentAngle = 0;

    public static double TargetAngle = 0;

    public double targetAngularVelocity = 0;

    public boolean Near = false;
    public boolean enabled = true;

    // Shot-on-the-move state.
    public boolean shotOnMove = false;

    // ==========================================
    // UTILITY
    // ==========================================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    /**
     * Finds the equivalent angle closest to the current
     * position while respecting mechanical limits.
     */
    private double getClosestValidAngle(double angle) {

        double bestAngle = clamp(angle, MIN_ANGLE, MAX_ANGLE);

        double bestDistance = Math.abs(bestAngle - currentAngle);

        for (int i = -3; i <= 3; i++) {

            double candidate = angle + (360.0 * i);

            if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) {
                continue;
            }

            double distance = Math.abs(candidate - currentAngle);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestAngle = candidate;
            }
        }

        return bestAngle;
    }

    // ==========================================
    // HIVE ALIGNMENT WITH GEOMETRIC COMPENSATION
    // ==========================================

    /**
     * Calculates the turret angle needed to point
     * toward the hive.
     *
     * Compensates for the turret pivot being displaced
     * from the robot's localization origin.
     */
    public double getHiveAngle() {

        Pose robotPose = robot.getRawPose();
        Pose hivePose = robot.getHive();

        double heading = robotPose.heading();

        // Robot center position.
        double robotX = robotPose.x();
        double robotY = robotPose.y();

        // Rotate turret offset into field coordinates.
        double turretOffsetFieldX =
                TURRET_OFFSET_X * Math.cos(heading)
                        - TURRET_OFFSET_Y * Math.sin(heading);

        double turretOffsetFieldY =
                TURRET_OFFSET_X * Math.sin(heading)
                        + TURRET_OFFSET_Y * Math.cos(heading);

        // Actual turret pivot position on the field.
        double turretX = robotX + turretOffsetFieldX;
        double turretY = robotY + turretOffsetFieldY;

        // Vector from turret pivot to hive.
        double deltaX = hivePose.x() - turretX;
        double deltaY = hivePose.y() - turretY;

        // Calculate absolute field angle.
        double fieldAngle = Math.toDegrees(
                Math.atan2(deltaY, deltaX)
        );

        // Convert to robot-relative angle.
        double robotHeading = Math.toDegrees(heading);

        double angle = fieldAngle - robotHeading;

        // Apply mechanical calibration correction.
        angle += robot.hiveIndex == 0
                ? HIVE_OFFSET
                : -HIVE_OFFSET;

        // Telemetry.
        robot.telemetry.addData("Robot X", robotX);
        robot.telemetry.addData("Robot Y", robotY);

        robot.telemetry.addData("Turret Pivot X", turretX);
        robot.telemetry.addData("Turret Pivot Y", turretY);

        robot.telemetry.addData("Hive X", hivePose.x());
        robot.telemetry.addData("Hive Y", hivePose.y());

        robot.telemetry.addData("Robot Heading", robotHeading);
        robot.telemetry.addData("Hive Field Angle", fieldAngle);
        robot.telemetry.addData("Hive Relative Angle", angle);

        return -angle;
    }

    // ==========================================
    // TARGET CONTROL
    // ==========================================

    /**
     * Sets a turret target without angular velocity FF.
     */
    public void setTargetAngle(double angle) {
        setTargetAngle(angle, 0);
    }

    /**
     * Sets a turret target and desired angular velocity.
     * Angular velocity is in radians per second.
     */
    public void setTargetAngle(double angle, double angularVelocity) {

        TargetAngle = getClosestValidAngle(angle);

        targetAngularVelocity = angularVelocity;

        double targetTicks = TargetAngle * TICKS_PER_DEGREE;

        turretController.setGoal(
                new KineticState(targetTicks, 0)
        );
    }

    // ==========================================
    // SHOT ON THE MOVE
    // ==========================================

    public void setShotOnMoveTarget(
            double angle,
            double angularVelocity
    ) {

        shotOnMove = true;
        AUTO_ALIGN = false;

        setTargetAngle(angle, angularVelocity);
    }

    public void clearShotOnMoveTarget() {

        shotOnMove = false;
        targetAngularVelocity = 0;

        AUTO_ALIGN = true;
    }

    // ==========================================
    // ALIGNMENT CONTROL
    // ==========================================

    public void disableAutoAlign() {
        AUTO_ALIGN = false;
    }

    public void alignToHive() {

        AUTO_ALIGN = true;
        shotOnMove = false;
    }

    // ==========================================
    // TURRET PID CONTROL
    // ==========================================

    private void update() {

        if (!enabled) {
            robot.turretMotor.set(0);
            return;
        }

        double currentPosition =
                robot.turretEncoder.getPosition();

        double pidOutput = turretController.calculate(
                new KineticState(currentPosition, 0)
        );

        double velocityOutput =
                velFF * targetAngularVelocity;

        double output = pidOutput + velocityOutput;

        robot.turretMotor.set(clamp(output, -1, 1));
    }

    // ==========================================
    // TURRET UPDATE
    // ==========================================

    public void updateTurret() {

        currentAngle =
                robot.turretEncoder.getPosition()
                        / TICKS_PER_DEGREE;

        Near = Math.abs(TargetAngle - currentAngle) <= TOLERANCE;

        update();
    }

    // ==========================================
    // ENABLE / DISABLE
    // ==========================================

    public void stop() {

        enabled = false;
        targetAngularVelocity = 0;

        robot.turretMotor.set(0);
    }

    public void enable() {
        enabled = true;
    }

    // ==========================================
    // PERIODIC
    // ==========================================

    @Override
    public void periodic() {

        if (!enabled) {
            robot.turretMotor.set(0);
            return;
        }

        // Automatic alignment must not overwrite
        // a shot-on-the-move target.
        if (AUTO_ALIGN && !shotOnMove) {
            setTargetAngle(getHiveAngle());
        }

        updateTurret();

        robot.telemetry.addData("Turret Target", TargetAngle);
        robot.telemetry.addData("Turret Current", currentAngle);
        robot.telemetry.addData("Turret Near", Near);

        robot.telemetry.addData(
                "Turret Velocity FF",
                targetAngularVelocity
        );
    }
}
