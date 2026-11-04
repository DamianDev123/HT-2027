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

    // Pollen lands on the floor.
    public static double hiveHeight = 0.0;

    // Shooter exit height above the floor.
    public static double shooterHeight = 6.0;

    // Positive means shooting upward.
    public static double shootingAngle = 15.0;

    // Gravity in inches per second squared.
    public static double gravity = 386.09;

    // Calculated projectile exit velocity.
    public static double exitVelocity = 0;

    // Calculated projectile flight time.
    public static double flightTime = 0;

    // ==========================================
    // EXIT VELOCITY CALIBRATION
    // ==========================================

    /*
     * Relationship:
     *
     * exitVelocity = calibrationSlope * motorSpeed
     *                + calibrationIntercept
     *
     * Inverse:
     *
     * motorSpeed = (exitVelocity - calibrationIntercept)
     *              / calibrationSlope
     *
     * Enter the coefficients calculated from your
     * Pollen calibration measurements.
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
     * to reach the floor at the specified horizontal distance.
     *
     * All distances are inches.
     * Returned velocity is inches per second.
     *
     * Assumes:
     * - Shooter launches upward.
     * - Pollen lands at floor level.
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
     * Calculates flight time from horizontal distance
     * and initial projectile velocity.
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
     * required launcher motor velocity.
     *
     * Uses the linear calibration from your
     * Pollen calibration tuner.
     */

    public double calculateMotorSpeed(double targetExitVelocity) {

        // ======================================
        // PROJECTILE SETTINGS
        // ======================================

        double angle = Math.toRadians(shootingAngle);

        // ======================================
        // CALCULATE EQUIVALENT FLOOR DISTANCE
        // ======================================

        // Vertical projectile equation:
        // 0 = shooterHeight + vY*t - (g*t^2)/2

        double verticalVelocity =
                targetExitVelocity * Math.sin(angle);

        double discriminant =
                verticalVelocity * verticalVelocity
                        + 2.0 * gravity * shooterHeight;

        if (targetExitVelocity <= 0 ||
                discriminant < 0 ||
                Double.isNaN(discriminant) ||
                Double.isInfinite(discriminant)) {
            return 0;
        }

        double flightTime =
                (verticalVelocity + Math.sqrt(discriminant))
                        / gravity;

        // Horizontal distance traveled before landing.
        double floorDistance =
                targetExitVelocity * Math.cos(angle) * flightTime;

        if (floorDistance <= 0 ||
                Double.isNaN(floorDistance) ||
                Double.isInfinite(floorDistance)) {
            return 0;
        }

        // ======================================
        // INVERT YOUR QUADRATIC CALIBRATION
        // ======================================

        // d = a*S^2 + b*S + c

        double a = 0.0000012987;
        double b = 0.0887706;
        double c = -59.1645;

        double quadraticDiscriminant =
                b * b - 4.0 * a * (c - floorDistance);

        if (quadraticDiscriminant < 0 ||
                Double.isNaN(quadraticDiscriminant) ||
                Double.isInfinite(quadraticDiscriminant)) {
            return 0;
        }

        double motorSpeed =
                (-b + Math.sqrt(quadraticDiscriminant))
                        / (2.0 * a);

        if (motorSpeed <= 0 ||
                Double.isNaN(motorSpeed) ||
                Double.isInfinite(motorSpeed)) {
            return 0;
        }

        // ======================================
        // TELEMETRY
        // ======================================

        robot.telemetry.addData(
                "Target Exit Velocity",
                targetExitVelocity
        );

        robot.telemetry.addData(
                "Equivalent Floor Distance",
                floorDistance
        );

        robot.telemetry.addData(
                "Calculated Motor Speed",
                motorSpeed
        );

        return Math.min(
                motorSpeed,
                Constants.LAUNCHER_MAX_VELOCITY
        );
    }

    // ==========================================
    // NORMAL VELOCITY CONTROL
    // ==========================================

    /*public void setGoal(double targetSpeed) {
        setGoal(targetSpeed, 0);
    }*/

    public void setGoal(
            double targetSpeed,
            double targetAcceleration
    ) {

        double clampedSpeed = Math.min(
                Math.abs(targetSpeed),
                Constants.LAUNCHER_MAX_VELOCITY
        );

        // Preserve original negative velocity convention.
        double signedVelocity = -clampedSpeed;

        double signedAcceleration = targetSpeed == 0 ? 0 : -targetAcceleration;

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

        // Preserve original encoder convention.
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

        // ======================================
        // MANUAL MODE
        // ======================================

        if (manual) {

            setGoal(manualSpeed, 0);

        }

        // ======================================
        // SHOOTING WHILE MOVING
        // ======================================

        else if (shotOnMove) {

            setGoal(
                    shotSpeed,
                    shotAcceleration
            );

            servoPos = shotHood;

        }

        // ======================================
        // PHYSICS-BASED DISTANCE SHOOTING
        // ======================================

        else {

            goalDistance = robot.getGoalDistance();

            // Calculate required Pollen exit velocity.
            exitVelocity = calculateExitVelocity(goalDistance);

            // Calculate estimated projectile flight time.
            flightTime = calculateFlightTime(
                    goalDistance,
                    exitVelocity
            );
            robot.telemetry.addData("TargetExit",exitVelocity);

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
                "Launcher Distance (in)",
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
                "Calibration Slope",
                calibrationSlope
        );

        robot.telemetry.addData(
                "Calibration Intercept",
                calibrationIntercept
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
                "Shooting Angle (degrees)",
                shootingAngle
        );

        robot.telemetry.addData(
                "Launcher SOTM",
                shotOnMove
        );
    }
}