
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

    private final Robot robot = Robot.getInstance();

    // ==========================================
    // PID CONTROL
    // ==========================================

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

    // ==========================================
    // POSITION-DEPENDENT HIVE OFFSET
    // ==========================================

    // These are calibration corrections based on
    // the robot's position around the hive.
    //
    // They are NOT based on robot heading.
    //
    // Position angle:
    // 0   = robot is to the right of hive
    // 90  = robot is above hive
    // 180 = robot is to the left of hive
    //
    // Values are in degrees.

    public static double OFFSET_0_DEG = -9;
    public static double OFFSET_45_DEG = -9;
    public static double OFFSET_90_DEG = -9;
    public static double OFFSET_135_DEG = -9;
    public static double OFFSET_180_DEG = -9;

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

    public boolean shotOnMove = false;

    // ==========================================
    // UTILITY
    // ==========================================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    private double wrapDegrees(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }
    public static double getHiveOffset(double robotX, double robotY) {

        Robot robot = Robot.getInstance();

        Pose hivePose = robot.getHive();

        // Position relative to hive.
        double deltaX = robotX - hivePose.x();
        double deltaY = robotY - hivePose.y();

        // Position angle around hive.
        // Independent of robot heading.
        double positionAngle = Math.toDegrees(
                Math.atan2(deltaY, deltaX)
        );

        positionAngle = Math.abs(
                ((positionAngle % 360) + 540) % 360 - 180
        );

        double[] angles = {
                0, 45, 90, 135, 180
        };

        double[] offsets = {
                OFFSET_0_DEG,
                OFFSET_45_DEG,
                OFFSET_90_DEG,
                OFFSET_135_DEG,
                OFFSET_180_DEG
        };

        double offset;

        if (positionAngle >= 180) {
            offset = offsets[4];
        } else {

            offset = offsets[0];

            for (int i = 0; i < angles.length - 1; i++) {

                if (positionAngle >= angles[i]
                        && positionAngle <= angles[i + 1]) {

                    double t = (positionAngle - angles[i])
                            / (angles[i + 1] - angles[i]);

                    offset = offsets[i]
                            + t * (offsets[i + 1] - offsets[i]);

                    break;
                }
            }
        }

        return robot.hiveIndex == 0
                ? offset
                : -offset;
    }

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
    // POSITION-DEPENDENT OFFSET INTERPOLATION
    // ==========================================

    /**
     * Calculates the optimal correction based on
     * the robot's position angle around the hive.
     *
     * Robot heading is not used.
     */
    private double getOptimalHiveOffset(double positionAngle) {

        positionAngle = Math.abs(wrapDegrees(positionAngle));

        double[] angles = {
                0, 45, 90, 135, 180
        };

        double[] offsets = {
                OFFSET_0_DEG,
                OFFSET_45_DEG,
                OFFSET_90_DEG,
                OFFSET_135_DEG,
                OFFSET_180_DEG
        };

        if (positionAngle <= 0) {
            return offsets[0];
        }

        if (positionAngle >= 180) {
            return offsets[4];
        }

        for (int i = 0; i < angles.length - 1; i++) {

            if (positionAngle >= angles[i]
                    && positionAngle <= angles[i + 1]) {

                double t = (positionAngle - angles[i])
                        / (angles[i + 1] - angles[i]);

                return offsets[i]
                        + t * (offsets[i + 1] - offsets[i]);
            }
        }

        return offsets[0];
    }

    // ==========================================
    // TURRET PIVOT GEOMETRY
    // ==========================================

    private double getTurretPivotX(Pose pose) {

        double heading = pose.heading();

        return pose.x()
                + TURRET_OFFSET_X * Math.cos(heading)
                - TURRET_OFFSET_Y * Math.sin(heading);
    }

    private double getTurretPivotY(Pose pose) {

        double heading = pose.heading();

        return pose.y()
                + TURRET_OFFSET_X * Math.sin(heading)
                + TURRET_OFFSET_Y * Math.cos(heading);
    }

    // ==========================================
    // HIVE ANGLE CALCULATION
    // ==========================================

    /**
     * Calculates the turret target using:
     *
     * 1. Robot position relative to hive.
     * 2. Position-dependent shooting correction.
     * 3. Robot heading for turret-relative conversion.
     * 4. Actual turret pivot position.
     */
    public double getHiveAngle() {

        Pose robotPose = robot.getRawPose();
        Pose hivePose = robot.getHive();

        // ======================================
        // ROBOT POSITION RELATIVE TO HIVE
        // ======================================

        double deltaX = robotPose.x() - hivePose.x();
        double deltaY = robotPose.y() - hivePose.y();

        // Position angle around hive.
        // Independent of robot heading.

        double positionAngle = Math.toDegrees(
                Math.atan2(deltaY, deltaX)
        );

        positionAngle = wrapDegrees(positionAngle);

        // ======================================
        // FIND OPTIMAL CORRECTION
        // ======================================

        double optimalOffset =
                getOptimalHiveOffset(positionAngle);

        // ======================================
        // ACTUAL TURRET PIVOT POSITION
        // ======================================

        double turretX = getTurretPivotX(robotPose);
        double turretY = getTurretPivotY(robotPose);

        // Vector from turret pivot to hive.
        double turretDeltaX = hivePose.x() - turretX;
        double turretDeltaY = hivePose.y() - turretY;

        // ======================================
        // FIELD-RELATIVE AIMING ANGLE
        // ======================================

        double fieldAngle = Math.toDegrees(
                Math.atan2(turretDeltaY, turretDeltaX)
        );

        // Convert to robot-relative angle.
        double robotHeading = Math.toDegrees(
                robotPose.heading()
        );

        double angle = fieldAngle - robotHeading;

        // Apply position-dependent correction.
        angle += robot.hiveIndex == 0
                ? optimalOffset
                : -optimalOffset;

        // ======================================
        // TELEMETRY
        // ======================================

        robot.telemetry.addData(
                "Hive Position Angle",
                positionAngle
        );

        robot.telemetry.addData(
                "Optimal Hive Offset",
                optimalOffset
        );

        robot.telemetry.addData(
                "Hive Field Angle",
                fieldAngle
        );

        robot.telemetry.addData(
                "Robot Heading",
                robotHeading
        );

        robot.telemetry.addData(
                "Turret Pivot X",
                turretX
        );

        robot.telemetry.addData(
                "Turret Pivot Y",
                turretY
        );

        robot.telemetry.addData(
                "Hive Turret Target",
                -angle
        );

        return -angle;
    }

    // ==========================================
    // TARGET CONTROL
    // ==========================================

    public void setTargetAngle(double angle) {
        setTargetAngle(angle, 0);
    }

    public void setTargetAngle(
            double angle,
            double angularVelocity
    ) {

        TargetAngle = getClosestValidAngle(angle);

        targetAngularVelocity = angularVelocity;

        double targetTicks =
                TargetAngle * TICKS_PER_DEGREE;

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
    // PID UPDATE
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

        robot.turretMotor.set(
                clamp(output, -1, 1)
        );
    }

    // ==========================================
    // TURRET UPDATE
    // ==========================================

    public void updateTurret() {

        currentAngle =
                robot.turretEncoder.getPosition()
                        / TICKS_PER_DEGREE;

        Near = Math.abs(
                TargetAngle - currentAngle
        ) <= TOLERANCE;

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

        robot.telemetry.addData(
                "Turret Target",
                TargetAngle
        );

        robot.telemetry.addData(
                "Turret Current",
                currentAngle
        );

        robot.telemetry.addData(
                "Turret Near",
                Near
        );

        robot.telemetry.addData(
                "Turret Velocity FF",
                targetAngularVelocity
        );
    }
}
