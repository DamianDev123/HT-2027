package org.firstinspires.ftc.teamcode.Solvers.Opmodes.Tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;

@Configurable
@TeleOp(name = "Pollen Calibration")
public class LauncherVelocityTuner extends OpMode {

    // ---------------- CALIBRATION SETTINGS ----------------

    public static double MIN_SPEED = 1100;
    public static double MAX_SPEED = 2000;

    public static double SHOOTER_HEIGHT = 50.0; // Inches
    public static double SHOOTING_ANGLE = -15.0; // Degrees
    public static double GRAVITY = 386.09; // Inches per second squared

    // Enter the measured horizontal distance through Panels
    public static double measuredRange = 0.0;

    // ---------------- CALIBRATION DATA ----------------

    private static final int NUM_TESTS = 10;

    private final double[] motorVelocitySamples =
            new double[NUM_TESTS];

    private final double[] exitVelocitySamples =
            new double[NUM_TESTS];

    private final double[] motorSpeeds =
            new double[NUM_TESTS];

    private int currentTest = 0;
    private double currentMotorSpeed = 0;

    private boolean lastA = false;
    private boolean calibrationComplete = false;

    private final Robot robot = Robot.getInstance();

    // ---------------- INITIALIZATION ----------------

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        initializeMotorSpeeds();

        Launcher.manual = false;
        Launcher.enabled = true;

        telemetry.addLine("POLLEN EXIT VELOCITY CALIBRATION");
        telemetry.addLine();
        telemetry.addLine("Enter the measured range through Panels.");
        telemetry.addLine("Press GAMEPAD 1 A to submit each measurement.");
        telemetry.addLine();
        telemetry.addData("Shooter Height", SHOOTER_HEIGHT);
        telemetry.addData("Shooting Angle", SHOOTING_ANGLE);
        telemetry.addData("Gravity", GRAVITY);
        telemetry.update();
    }

    @Override
    public void start() {
        startCurrentTest();
    }

    // ---------------- MAIN LOOP ----------------

    @Override
    public void loop() {

        if (calibrationComplete) {
            disableTurret();

            telemetry.addLine("CALIBRATION COMPLETE");
            telemetry.addLine("See results below.");
            telemetry.update();
            return;
        }

        // Run the normal robot loop.
        // This also updates the launcher through the scheduler.
        robot.updateLoop();

        // Keep the turret disabled.
        disableTurret();

        // Detect a new A button press.
        boolean currentA = gamepad1.a;

        if (currentA && !lastA) {
            submitMeasurement();
        }

        lastA = currentA;

        // ---------------- TELEMETRY ----------------

        telemetry.addLine("POLLEN CALIBRATION");

        telemetry.addData(
                "Test",
                "%d / %d",
                currentTest + 1,
                NUM_TESTS
        );

        telemetry.addData(
                "Motor Target",
                "%.2f",
                currentMotorSpeed
        );

        telemetry.addData(
                "Measured Range (in)",
                "%.2f",
                measuredRange
        );

        telemetry.addLine();
        telemetry.addLine("Enter range in Panels.");
        telemetry.addLine("Press A to submit measurement.");

        telemetry.addLine();

        telemetry.addData(
                "Shooter Height",
                "%.2f in",
                SHOOTER_HEIGHT
        );

        telemetry.addData(
                "Shooting Angle",
                "%.2f degrees",
                SHOOTING_ANGLE
        );

        telemetry.addLine();

        for (int i = 0; i < currentTest; i++) {
            telemetry.addData(
                    "Sample " + (i + 1),
                    "Motor: %.2f | Exit: %.2f in/s",
                    motorVelocitySamples[i],
                    exitVelocitySamples[i]
            );
        }

        telemetry.update();
    }

    // ---------------- DISABLE TURRET ----------------

    private void disableTurret() {

        robot.turret.disableAutoAlign();
        robot.turret.clearShotOnMoveTarget();

        Robot.shootingWhileMovingEnabled = false;

        // Force turret motor output to zero.
        robot.turretMotor.set(0);
    }

    // ---------------- MOTOR SPEEDS ----------------

    private void initializeMotorSpeeds() {

        double increment =
                (MAX_SPEED - MIN_SPEED) / (NUM_TESTS - 1);

        for (int i = 0; i < NUM_TESTS; i++) {
            motorSpeeds[i] = MIN_SPEED + increment * i;
        }
    }

    private void startCurrentTest() {

        if (currentTest >= NUM_TESTS) {
            calibrationComplete = true;
            return;
        }

        currentMotorSpeed = motorSpeeds[currentTest];

        measuredRange = 0;

        Launcher.enabled = true;
        Launcher.manual = true;
        Launcher.manualSpeed = currentMotorSpeed;
    }

    // ---------------- SUBMIT MEASUREMENT ----------------

    private void submitMeasurement() {

        if (currentTest >= NUM_TESTS) {
            return;
        }

        if (measuredRange <= 0) {
            telemetry.addLine(
                    "Invalid range. Enter a distance greater than zero."
            );
            telemetry.update();
            return;
        }

        double exitVelocity = calculateExitVelocity(
                measuredRange,
                SHOOTER_HEIGHT
        );

        if (exitVelocity <= 0 ||
                Double.isNaN(exitVelocity) ||
                Double.isInfinite(exitVelocity)) {

            telemetry.addLine("Invalid physics result.");
            telemetry.addLine(
                    "Check the measured range and shooter height."
            );
            telemetry.update();
            return;
        }

        // Store calibration sample.
        motorVelocitySamples[currentTest] = currentMotorSpeed;
        exitVelocitySamples[currentTest] = exitVelocity;

        currentTest++;

        measuredRange = 0;

        if (currentTest < NUM_TESTS) {
            startCurrentTest();
        } else {
            calibrationComplete = true;
            calculateRegression();
        }
    }

    // ---------------- PROJECTILE PHYSICS ----------------

    /**
     * Calculates initial projectile velocity using:
     *
     * v = sqrt(
     *     g * d^2 /
     *     (2 * cos(theta)^2 * (d * tan(theta) + h))
     * )
     *
     * d = horizontal distance
     * h = shooter height above ground
     * theta = shooting angle
     * g = gravitational acceleration
     *
     * All distances are in inches.
     * Result is inches per second.
     */
    public static double calculateExitVelocity(
            double distance,
            double shooterHeight
    ) {

        double angle = Math.toRadians(SHOOTING_ANGLE);

        double denominator =
                2.0
                        * Math.pow(Math.cos(angle), 2)
                        * (distance * Math.tan(angle) + shooterHeight);

        if (distance <= 0 || denominator <= 0) {
            return 0;
        }

        double velocitySquared =
                (GRAVITY * distance * distance) / denominator;

        if (velocitySquared <= 0 ||
                Double.isNaN(velocitySquared) ||
                Double.isInfinite(velocitySquared)) {
            return 0;
        }

        return Math.sqrt(velocitySquared);
    }

    // ---------------- LINEAR REGRESSION ----------------

    private void calculateRegression() {

        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;

        for (int i = 0; i < NUM_TESTS; i++) {

            double x = motorVelocitySamples[i];
            double y = exitVelocitySamples[i];

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double n = NUM_TESTS;

        double denominator =
                n * sumXX - sumX * sumX;

        if (denominator == 0) {
            telemetry.addLine(
                    "Regression failed: invalid motor speed data."
            );
            telemetry.update();
            return;
        }

        // y = mx + b

        double slope =
                (n * sumXY - sumX * sumY) / denominator;

        double intercept =
                (sumY - slope * sumX) / n;

        // ---------------- R-SQUARED ----------------

        double meanY = sumY / n;

        double totalSumSquares = 0;
        double residualSumSquares = 0;

        for (int i = 0; i < NUM_TESTS; i++) {

            double actual = exitVelocitySamples[i];

            double predicted =
                    slope * motorVelocitySamples[i] + intercept;

            totalSumSquares += Math.pow(actual - meanY, 2);

            residualSumSquares +=
                    Math.pow(actual - predicted, 2);
        }

        double rSquared =
                totalSumSquares == 0
                        ? 1.0
                        : 1.0 - residualSumSquares / totalSumSquares;

        // ---------------- RESULTS ----------------

        telemetry.clear();

        telemetry.addLine("CALIBRATION COMPLETE");
        telemetry.addLine();

        telemetry.addLine("EXIT VELOCITY EQUATION");

        telemetry.addData(
                "Equation",
                "v = %.6f * motorSpeed + %.6f",
                slope,
                intercept
        );

        telemetry.addData("Slope", slope);
        telemetry.addData("Intercept", intercept);
        telemetry.addData("R Squared", rSquared);

        telemetry.addLine();
        telemetry.addLine("CALIBRATION SAMPLES");

        for (int i = 0; i < NUM_TESTS; i++) {

            telemetry.addData(
                    "Test " + (i + 1),
                    "Motor: %.2f | Exit: %.2f in/s",
                    motorVelocitySamples[i],
                    exitVelocitySamples[i]
            );
        }

        telemetry.update();
    }

    // ---------------- STOP ----------------

    @Override
    public void stop() {

        Launcher.manual = false;

        robot.launcher.stop();

        disableTurret();

        robot.turretMotor.set(0);
    }
}