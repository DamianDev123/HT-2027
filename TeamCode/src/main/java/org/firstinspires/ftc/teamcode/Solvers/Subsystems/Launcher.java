
package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;

@Configurable
public class Launcher extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    // ==========================================
    // PIDF CONFIGURATION
    // ==========================================

    public static PIDCoefficients testPid =
            new PIDCoefficients(0.015, 0, 0);

    // Constructor order: kV, kA, kS
    public static BasicFeedforwardParameters timeUnit =
            new BasicFeedforwardParameters(
                    0.00032,
                    0.0,
                    0.09
            );

    public static ControlSystem spinner =
            ControlSystem.builder()
                    .velPid(testPid)
                    .basicFF(timeUnit)
                    .build();

    // ==========================================
    // HYBRID CONTROLLER CONFIGURATION
    // ==========================================

    public static double maxPower = 1.0;

    public static double bangBangThreshold = 50;

    public static double velocityTolerance = 30;

    // ==========================================
    // LAUNCHER SETTINGS
    // ==========================================

    public static double servoPos = 0.65;

    public static double speed = 1500;

    public static double goalDistance = 0;

    public static double manualSpeed = 1500;

    public static boolean manual = false;

    public static boolean enabled = true;

    public static double hoodMinimum = 0.0;

    // ==========================================
    // PROJECTILE PHYSICS
    // ==========================================

    // Hive target height above ground (inches).
    public static double hiveHeight = 50.0;

    // Shooter exit height above ground (inches).
    public static double shooterHeight = 8.0;

    // Shooting angle relative to horizontal (degrees).
    public static double shootingAngle = -15.0;

    // Gravity in inches per second squared.
    public static double gravity = 386.09;

    // Calculated projectile exit velocity (inches/second).
    public static double exitVelocity = 0;

    // Calculated projectile flight time (seconds).
    public static double flightTime = 0;

    // ==========================================
    // EXIT VELOCITY CALIBRATION
    // ==========================================

    /*
     * Calibration equation:
     *
     * exitVelocity = calibrationSlope * motorSpeed
     *                + calibrationIntercept
     *
     * Inverse:
     *
     * motorSpeed = (exitVelocity - calibrationIntercept)
     *              / calibrationSlope
     *
     * Replace these values with the results from
     * your 10-speed Pollen calibration.
     */

    public static double calibrationSlope = 0.0;

    public static double calibrationIntercept = 0.0;

    // ==========================================
    // SHOOTING WHILE MOVING
    // ==========================================

    private boolean shotOnMove = false;

    private double shotSpeed = 0;

    private double shotAcceleration = 0;

    private double shotHood = 0.65;

    // ==========================================
    // HYBRID STATE
    // ==========================================

    private boolean bangBangActive = false;

    private double motorOutput = 0;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================

    public Launcher() {
        spinner.setGoal(new KineticState(0, 0, 0));
    }

    // ==========================================
    // PROJECTILE EXIT VELOCITY
    // ==========================================

    /**
     * Calculates the required initial Pollen velocity
     * to reach the hive.
     *
     * All distances are inches.
     * Returned velocity is inches per second.
     *
     * Uses:
     *
     * v = sqrt(
     *     g * d^2 /
     *     (2 * cos(theta)^2 * (d * tan(theta) - deltaY))
     * )
     */

    public double calculateExitVelocity(double distance) {

        double angle = Math.toRadians(shootingAngle);

        double deltaY = hiveHeight - shooterHeight;

        double denominator =
                2.0
                        * Math.pow(Math.cos(angle), 2)
                        * (distance * Math.tan(angle) - deltaY);

        if (distance <= 0 || denominator <= 0) {
            return 0;
        }

        double velocitySquared =
                (gravity * distance * distance) / denominator;

        if (velocitySquared <= 0 ||
                Double.isNaN(velocitySquared) ||
                Double.isInfinite(velocitySquared)) {
            return 0;
        }

        return Math.sqrt(velocitySquared);
    }

    // ==========================================
    // PROJECTILE FLIGHT TIME
    // ==========================================

    /**
     * Calculates the time needed for the Pollen
     * to travel the horizontal distance.
     *
     * Distance: inches
     * Exit velocity: inches/second
     * Return: seconds
     */

    public double calculateFlightTime(
            double distance,
            double initialVelocity
    ) {

        double angle = Math.toRadians(shootingAngle);

        double horizontalVelocity =
                initialVelocity * Math.cos(angle);

        if (distance <= 0 || horizontalVelocity <= 0) {
            return 0;
        }

        return distance / horizontalVelocity;
    }

    /**
     * Calculates flight time automatically using
     * the required exit velocity.
     */

    public double calculateFlightTime(double distance) {

        double velocity = calculateExitVelocity(distance);

        if (velocity <= 0) {
            return 0;
        }

        return calculateFlightTime(distance, velocity);
    }

    // ==========================================
    // MOTOR SPEED CONVERSION
    // ==========================================

    /**
     * Converts projectile exit velocity into the
     * required launcher motor velocity using the
     * calibration equation.
     *
     * Returns 0 if calibration has not been entered.
     */

    public double calculateMotorSpeed(double targetExitVelocity) {

        if (calibrationSlope == 0 ||
                Double.isNaN(calibrationSlope) ||
                Double.isInfinite(calibrationSlope)) {
            return 0;
        }

        double motorSpeed =
                (targetExitVelocity - calibrationIntercept)
                        / calibrationSlope;

        if (motorSpeed <= 0 ||
                Double.isNaN(motorSpeed) ||
                Double.isInfinite(motorSpeed)) {
            return 0;
        }

        return Math.min(
                motorSpeed,
                Constants.LAUNCHER_MAX_VELOCITY
        );
    }

    // ==========================================
    // NORMAL VELOCITY CONTROL
    // ==========================================

    public void setGoal(double targetSpeed) {
        setGoal(targetSpeed, 0);
    }

    public void setGoal(
            double targetSpeed,
            double targetAcceleration
    ) {

        double clampedSpeed = Math.min(
                Math.abs(targetSpeed),
                Constants.LAUNCHER_MAX_VELOCITY
        );

        // Original negative velocity convention.
        double signedVelocity = -clampedSpeed;

        double signedAcceleration =
                targetSpeed == 0
                        ? 0
                        : -targetAcceleration;

        spinner.setGoal(
                new KineticState(
                        0,
                        signedVelocity,
                        signedAcceleration
                )
        );
    }

    // ==========================================
    // SHOOTING WHILE MOVING
    // ==========================================

    public void setShotOnMoveTarget(
            double targetSpeed,
            double targetAcceleration,
            double hoodPosition
    ) {

        shotOnMove = true;

        shotSpeed = targetSpeed;
        shotAcceleration = targetAcceleration;
        shotHood = hoodPosition;
    }

    public void clearShotOnMoveTarget() {

        shotOnMove = false;

        shotSpeed = 0;
        shotAcceleration = 0;
    }

    public boolean isShotOnMove() {
        return shotOnMove;
    }

    // ==========================================
    // ENABLE / DISABLE
    // ==========================================

    public void enable() {
        enabled = true;
    }

    public void stop() {

        enabled = false;

        shotOnMove = false;

        shotSpeed = 0;
        shotAcceleration = 0;

        bangBangActive = false;
        motorOutput = 0;

        spinner.setGoal(new KineticState(0, 0, 0));

        robot.launchMotor.set(0);
    }

    // ==========================================
    // HYBRID PIDF + BANG-BANG CONTROLLER
    // ==========================================

    public void update() {

        // Preserve your working encoder convention.
        double currentVelocity =
                -robot.launchEncoder.getCorrectedVelocity();

        // Target velocity is stored as negative.
        double targetVelocity =
                spinner.getGoal().getVelocity();

        // Convert to positive magnitudes.
        double targetSpeed = -targetVelocity;

        double error = targetSpeed - currentVelocity;

        // ======================================
        // ZERO TARGET
        // ======================================

        if (targetSpeed <= 0) {

            bangBangActive = false;
            motorOutput = 0;

            spinner.setGoal(new KineticState(0, 0, 0));

            robot.launchMotor.set(0);
            return;
        }

        // ======================================
        // MODE SWITCHING
        // ======================================

        if (!bangBangActive &&
                error > bangBangThreshold) {

            bangBangActive = true;
        }

        if (bangBangActive &&
                error <= velocityTolerance) {

            bangBangActive = false;
        }

        // ======================================
        // BANG-BANG MODE
        // ======================================

        if (bangBangActive) {

            motorOutput = -maxPower;

        }

        // ======================================
        // PIDF MODE
        // ======================================

        else {

            motorOutput = spinner.calculate(
                    new KineticState(
                            0,
                            robot.launchEncoder.getCorrectedVelocity(),
                            0
                    )
            );

            motorOutput = Math.max(
                    -maxPower,
                    Math.min(motorOutput, maxPower)
            );
        }

        robot.launchMotor.set(motorOutput);
    }

    // ==========================================
    // PERIODIC
    // ==========================================

    @Override
    public void periodic() {

        if (!enabled) {

            spinner.setGoal(new KineticState(0, 0, 0));

            bangBangActive = false;
            motorOutput = 0;

            robot.launchMotor.set(0);

            return;
        }

        if (manual) {

            setGoal(manualSpeed, 0);

        } else if (shotOnMove) {

            setGoal(
                    shotSpeed,
                    shotAcceleration
            );

            servoPos = shotHood;

        } else {

            // ==================================
            // PHYSICS-BASED DISTANCE SHOOTING
            // ==================================

            goalDistance = robot.getGoalDistance();

            // Calculate required Pollen exit velocity.
            exitVelocity = calculateExitVelocity(goalDistance);

            // Calculate estimated flight time.
            flightTime = calculateFlightTime(
                    goalDistance,
                    exitVelocity
            );

            // Convert projectile velocity into motor velocity.
            speed = calculateMotorSpeed(exitVelocity);

            // Only command the launcher if calibration
            // coefficients have been entered.
            if (speed > 0) {
                setGoal(speed, 0);
            } else {
                setGoal(0, 0);
            }
        }

        // ======================================
        // HOOD CONTROL
        // ======================================

        servoPos = Math.max(
                servoPos,
                hoodMinimum
        );

        robot.hoodServo.set(servoPos);

        // ======================================
        // HYBRID CONTROLLER
        // ======================================

        update();

        // ======================================
        // TELEMETRY
        // ======================================

        robot.telemetry.addData(
                "Launcher Target Velocity",
                spinner.getGoal().getVelocity()
        );

        robot.telemetry.addData(
                "Launcher Current Velocity",
                robot.launchEncoder.getCorrectedVelocity()
        );

        robot.telemetry.addData(
                "Launcher Motor Output",
                motorOutput
        );

        robot.telemetry.addData(
                "Launcher Controller",
                bangBangActive ? "BANG-BANG" : "PIDF"
        );

        robot.telemetry.addData(
                "Launcher Target Acceleration",
                spinner.getGoal().getAcceleration()
        );

        robot.telemetry.addData(
                "Launcher Distance",
                goalDistance
        );

        robot.telemetry.addData(
                "Required Exit Velocity (in/s)",
                exitVelocity
        );

        robot.telemetry.addData(
                "Projectile Flight Time (s)",
                flightTime
        );

        robot.telemetry.addData(
                "Calculated Motor Speed",
                speed
        );

        robot.telemetry.addData(
                "Calibration Available",
                calibrationSlope != 0
        );

        robot.telemetry.addData(
                "Hive Height (in)",
                hiveHeight
        );

        robot.telemetry.addData(
                "Shooter Height (in)",
                shooterHeight
        );

        robot.telemetry.addData(
                "Shooting Angle",
                shootingAngle
        );

        robot.telemetry.addData(
                "Launcher SOTM",
                shotOnMove
        );
    }
}
