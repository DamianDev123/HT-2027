
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
    // ORIGINAL PIDF CONFIGURATION
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

    // Enter Bang-Bang when below target by this amount.
    public static double bangBangThreshold = 50;

    // Return to PIDF when within this amount.
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

        // Original negative velocity convention.
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

        // Preserve your working encoder convention.
        double currentVelocity =
                -robot.launchEncoder.getCorrectedVelocity();

        // Target velocity is stored as negative.
        double targetVelocity =
                spinner.getGoal().getVelocity();

        // Convert to positive magnitudes for comparison.
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

        // Enter Bang-Bang when significantly below target.
        if (!bangBangActive &&
                error > bangBangThreshold) {

            bangBangActive = true;
        }

        // Return to PIDF when close to target.
        if (bangBangActive &&
                error <= velocityTolerance) {

            bangBangActive = false;
        }

        // ======================================
        // BANG-BANG MODE
        // ======================================

        if (bangBangActive) {

            // Same direction and output as your
            // previously working Bang-Bang controller.
            motorOutput = -maxPower;

        }

        // ======================================
        // ORIGINAL NEXTFTC PIDF MODE
        // ======================================

        else {

            motorOutput = spinner.calculate(
                    new KineticState(
                            0,
                            robot.launchEncoder.getCorrectedVelocity(),
                            0
                    )
            );

            // Limit output.
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

            // Distance-based shooting.
            goalDistance = robot.getGoalDistance();

            speed = 9.06593 * goalDistance
                    + 1158.79121;

            setGoal(speed, 0);
        }

        // Hood control.
        servoPos = Math.max(
                servoPos,
                hoodMinimum
        );

        robot.hoodServo.set(servoPos);

        // Hybrid controller.
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
                "Launcher SOTM",
                shotOnMove
        );
    }
}
