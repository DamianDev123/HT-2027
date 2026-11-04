
package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;

@Configurable
public class ShootingWhileMoving extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    // ---------------- CONFIGURATION ----------------

    // Velocity and acceleration low-pass filters
    public static double velocityFilterAlpha = 0.35;
    public static double accelerationFilterAlpha = 0.20;

    // Deadbands
    public static double VELOCITY_DEADBAND = 0.5;
    public static double ANGULAR_VELOCITY_DEADBAND = 200000.0;
    public static double ACCELERATION_DEADBAND = 20.0;


    // Sampling interval
    public static double SAMPLING_DT = 0.02;

    // Additional turret offset, in radians
    public static double turretOffset = 0.0;

    // ---------------- MOTION ESTIMATION ----------------

    private double filteredVx = 0;
    private double filteredVy = 0;
    private double filteredOmega = 0;

    private double filteredAx = 0;
    private double filteredAy = 0;

    private double previousVx = 0;
    private double previousVy = 0;

    private long previousTime = 0;

    private boolean motionInitialized = false;

    // ---------------- SHOT SOLUTION ----------------

    public static class ShotSolution {

        public final double distance;
        public final double tof;

        public final double turretAngle;
        public final double turretAngularVelocity;

        public final double distanceRate;

        public final double wheelSpeed;
        public final double wheelAcceleration;

        public final double robotVelocityX;
        public final double robotVelocityY;

        public final double robotAccelerationX;
        public final double robotAccelerationY;

        public final double predictedHeading;

        public ShotSolution(
                double distance,
                double tof,
                double turretAngle,
                double turretAngularVelocity,
                double distanceRate,
                double wheelSpeed,
                double wheelAcceleration,
                double robotVelocityX,
                double robotVelocityY,
                double robotAccelerationX,
                double robotAccelerationY,
                double predictedHeading
        ) {

            this.distance = distance;
            this.tof = tof;

            this.turretAngle = turretAngle;
            this.turretAngularVelocity = turretAngularVelocity;

            this.distanceRate = distanceRate;

            this.wheelSpeed = wheelSpeed;
            this.wheelAcceleration = wheelAcceleration;

            this.robotVelocityX = robotVelocityX;
            this.robotVelocityY = robotVelocityY;

            this.robotAccelerationX = robotAccelerationX;
            this.robotAccelerationY = robotAccelerationY;

            this.predictedHeading = predictedHeading;
        }
    }

    // ---------------- HELPERS ----------------

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    private double lerp(double current, double target, double alpha) {
        return current + alpha * (target - current);
    }

    public double getFlyTime(double distance) {

        return robot.launcher.calculateFlightTime(distance);
    }

    private double getWheelSpeed(double distance) {

        return robot.launcher.calculateMotorSpeed(robot.launcher.calculateExitVelocity(distance));
    }

    private double sampleRate(
            double distance,
            double distanceRate
    ) {

        double currentSpeed = getWheelSpeed(distance);

        double futureSpeed = getWheelSpeed(
                distance + distanceRate * SAMPLING_DT
        );

        return (futureSpeed - currentSpeed) / SAMPLING_DT;
    }

    // ---------------- MOTION UPDATE ----------------

    /**
     * Call once per robot loop, after follower.update().
     */
    public void updateMotionEstimate() {

        if (robot.follower == null) {
            return;
        }

        double measuredVx = robot.follower.velocity().vx;
        double measuredVy = robot.follower.velocity().vy;
        double measuredOmega = robot.follower.velocity().omega;

        long now = System.nanoTime();

        // Initialize motion estimation
        if (!motionInitialized) {

            filteredVx = measuredVx;
            filteredVy = measuredVy;
            filteredOmega = measuredOmega;

            previousVx = measuredVx;
            previousVy = measuredVy;

            previousTime = now;

            motionInitialized = true;

            return;
        }

        double dt = (now - previousTime) / 1e9;

        // Reject invalid time intervals
        if (dt <= 0 || dt > 0.2) {

            previousTime = now;

            previousVx = filteredVx;
            previousVy = filteredVy;

            return;
        }

        // ---------------- FILTER VELOCITY ----------------

        filteredVx = lerp(
                filteredVx,
                measuredVx,
                velocityFilterAlpha
        );

        filteredVy = lerp(
                filteredVy,
                measuredVy,
                velocityFilterAlpha
        );

        filteredOmega = lerp(
                filteredOmega,
                measuredOmega,
                velocityFilterAlpha
        );

        // ---------------- STATIONARY DETECTION ----------------

        double speed = Math.hypot(
                filteredVx,
                filteredVy
        );

        if (speed < VELOCITY_DEADBAND) {

            filteredVx = 0;
            filteredVy = 0;

            filteredAx = 0;
            filteredAy = 0;

            previousVx = 0;
            previousVy = 0;

            // Stop angular velocity noise too
            if (Math.abs(filteredOmega) < ANGULAR_VELOCITY_DEADBAND) {
                filteredOmega = 0;
            }

            previousTime = now;

            return;
        }

        // ---------------- CALCULATE ACCELERATION ----------------

        double measuredAx =
                (filteredVx - previousVx) / dt;

        double measuredAy =
                (filteredVy - previousVy) / dt;

        // ---------------- FILTER ACCELERATION ----------------

        filteredAx = lerp(
                filteredAx,
                measuredAx,
                accelerationFilterAlpha
        );

        filteredAy = lerp(
                filteredAy,
                measuredAy,
                accelerationFilterAlpha
        );

        // ---------------- ACCELERATION DEADBAND ----------------

        if (Math.abs(filteredAx) < ACCELERATION_DEADBAND) {
            filteredAx = 0;
        }

        if (Math.abs(filteredAy) < ACCELERATION_DEADBAND) {
            filteredAy = 0;
        }

        // ---------------- ANGULAR VELOCITY DEADBAND ----------------

        if (Math.abs(filteredOmega) < ANGULAR_VELOCITY_DEADBAND) {
            filteredOmega = 0;
        }

        // ---------------- SAVE PREVIOUS VALUES ----------------

        previousVx = filteredVx;
        previousVy = filteredVy;

        previousTime = now;
    }

    // ---------------- PREDICTED POSE ----------------

    public Pose getExpectedPose(Pose currentPose) {

        if (!motionInitialized) {
            return currentPose;
        }

        double distance = Math.hypot(
                robot.getHive().x() - currentPose.x(),
                robot.getHive().y() - currentPose.y()
        );

        double tof = getFlyTime(distance);

        double futureX =
                currentPose.x()
                        + filteredVx * tof
                        + 0.5 * filteredAx * tof * tof;

        double futureY =
                currentPose.y()
                        + filteredVy * tof
                        + 0.5 * filteredAy * tof * tof;

        double futureHeading =
                currentPose.heading() + filteredOmega * tof;

        return new Pose(
                futureX,
                futureY,
                futureHeading
        );
    }

    // ---------------- SHOT CALCULATION ----------------

    public ShotSolution calculate(Pose turretPose) {

        Pose goal = robot.getHive();

        // Current filtered robot motion
        double vx = filteredVx;
        double vy = filteredVy;
        double omega = filteredOmega;

        double ax = filteredAx;
        double ay = filteredAy;

        // ---------------- TIME OF FLIGHT ----------------

        double initialDistance = Math.hypot(
                goal.x() - turretPose.x(),
                goal.y() - turretPose.y()
        );

        double tof = getFlyTime(initialDistance);

        // ---------------- PREDICT FUTURE POSITION ----------------

        double futureX =
                turretPose.x()
                        + vx * tof
                        + 0.5 * ax * tof * tof;

        double futureY =
                turretPose.y()
                        + vy * tof
                        + 0.5 * ay * tof * tof;

        double futureHeading =
                turretPose.heading() + omega * tof;

        double dx = goal.x() - futureX;
        double dy = goal.y() - futureY;

        double r2 = dx * dx + dy * dy;

        double distance = Math.sqrt(r2);

        if (distance < 1e-6) {
            distance = 1e-6;
        }

        // ---------------- TURRET ANGLE ----------------

        // ---------------- TURRET ANGLE ----------------

        // Field-relative direction to hive.
        double fieldAngle = Math.atan2(dy, dx);

        // Calculate position-dependent offset using
        // predicted position at projectile arrival.
        //
        // This is NOT based on robot heading.
        double hiveOffset = Math.toRadians(
                Turret.getHiveOffset(futureX, futureY)
        );

        // Preserve your existing launcher adjustment.
        if (Launcher.servoPos > 0.72) {
            hiveOffset += Math.PI;
        }

        // Calculate turret target.
        double wantedTurretAngle =
                -(fieldAngle - futureHeading + hiveOffset)
                        + turretOffset;

        // ---------------- TURRET ANGULAR VELOCITY ----------------

        double lineOfSightRate =
                (dy * vx - dx * vy) / r2;

        double wantedTurretVelocity =
                omega - lineOfSightRate;

        // ---------------- DISTANCE RATE ----------------

        double distanceRate =
                (dx * vx + dy * vy) / distance;

        // ---------------- FLYWHEEL ----------------

        double wantedWheelSpeed = getWheelSpeed(distance);

        double wantedWheelAcceleration = sampleRate(
                distance,
                distanceRate
        );

        // ---------------- RETURN ----------------

        return new ShotSolution(
                distance,
                tof,
                Math.toDegrees(wantedTurretAngle),
                wantedTurretVelocity,
                distanceRate,
                wantedWheelSpeed,
                wantedWheelAcceleration,
                vx,
                vy,
                ax,
                ay,
                futureHeading
        );
    }

    // ---------------- TELEMETRY ----------------

    public void addTelemetry() {

        robot.telemetry.addData("SOTM Vx", filteredVx);
        robot.telemetry.addData("SOTM Vy", filteredVy);

        robot.telemetry.addData("SOTM Ax", filteredAx);
        robot.telemetry.addData("SOTM Ay", filteredAy);

        robot.telemetry.addData("SOTM Omega", filteredOmega);

        robot.telemetry.addData(
                "SOTM Speed",
                Math.hypot(filteredVx, filteredVy)
        );
    }
}
