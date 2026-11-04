
package org.firstinspires.ftc.teamcode.Globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.helpers.Drawing;

import java.io.File;
import java.util.List;
import java.util.Objects;
@Configurable
public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    private static final Robot instance = new Robot();

    public static Robot getInstance() {
        return instance;
    }

    // ---------------- HARDWARE ----------------

    public Follower follower;

    public MotorEx launchMotor;
    public Motor.Encoder launchEncoder;

    public MotorEx intakeMotor;
    public MotorEx transferMotor;

    public MotorEx frontLeft;
    public MotorEx frontRight;
    public MotorEx backLeft;
    public MotorEx backRight;

    public MotorEx turretMotor;
    public Motor.Encoder turretEncoder;

    public ServoEx hoodServo;
    public ServoEx flap;

    public DigitalChannel breakBeam1;

    // ---------------- SUBSYSTEMS ----------------

    public Launcher launcher;
    public Turret turret;
    public Storage storage;

    public ShootingWhileMoving sotm;

    // ---------------- OTHER ----------------

    public Telemetry telemetry;

    public File file;

    public LynxModule ex;
    public List<LynxModule> hubs;

    HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;

    public Pose[] hive = new Pose[2];
    public int hiveIndex = 0;

    // ---------------- SOTM ----------------

    public static boolean shootingWhileMovingEnabled = false;

    // ---------------- INITIALIZATION ----------------

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hwMap);

        initializedPoses();

        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");

        if (!logsFolder.exists()) {
            logsFolder.mkdirs();
        }

        long timestamp = System.currentTimeMillis();

        file = new File(
                logsFolder,
                "profiler-" + timestamp + ".csv"
        );

        hubs = hwMap.getAll(LynxModule.class);

        // Enable MANUAL bulk caching mode for ALL hubs
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ---------------- MOTORS ----------------

        launchMotor = new MotorEx(hwMap, "Shooter")
                .setCachingTolerance(0.01);

        launchMotor.setRunMode(Motor.RunMode.RawPower);

        launchMotor.setZeroPowerBehavior(
                Motor.ZeroPowerBehavior.FLOAT
        );

        launchEncoder = new Motor(hwMap, "Shooter").encoder;

        launchEncoder.setDirection(Motor.Direction.FORWARD);

        turretMotor = new MotorEx(hwMap, "Turret")
                .setCachingTolerance(0.01);

        turretEncoder = new Motor(hwMap, "Turret").encoder;

        intakeMotor = new MotorEx(hwMap, "Intake")
                .setCachingTolerance(0.01);

        transferMotor = new MotorEx(hwMap, "Transfer")
                .setCachingTolerance(0.01);

        intakeMotor.setInverted(true);

        intakeMotor.setZeroPowerBehavior(
                Motor.ZeroPowerBehavior.BRAKE
        );

        transferMotor.setInverted(true);

        transferMotor.setRunMode(Motor.RunMode.RawPower);

        transferMotor.setZeroPowerBehavior(
                Motor.ZeroPowerBehavior.BRAKE
        );

        // ---------------- SERVOS ----------------

        flap = new ServoEx(hwMap, "Flap")
                .setCachingTolerance(0.001);

        hoodServo = new ServoEx(hwMap, "Hood")
                .setCachingTolerance(0.001);

        breakBeam1 = hwMap.get(
                DigitalChannel.class,
                "transferBreakBeam"
        );

        // ---------------- SUBSYSTEMS ----------------

        launcher = new Launcher();

        turret = new Turret();

        storage = new Storage();

        sotm = new ShootingWhileMoving();

        CommandScheduler.getInstance().registerSubsystem(
                launcher,
                turret
        );

        voltageSensor = hwMap.voltageSensor;

        Drawing.init();
    }

    // ---------------- BATTERY ----------------

    public double getBatteryVoltage() {

        double result = Double.POSITIVE_INFINITY;

        if (voltageSensor == null) {
            return 12.0;
        }

        for (VoltageSensor sensor : voltageSensor) {

            double voltage = sensor.getVoltage();

            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        return result == Double.POSITIVE_INFINITY ? 12.0 : result;
    }

    // ---------------- HUBS ----------------

    @NonNull
    public static LynxModule getExpansionHub(List<LynxModule> hubs) {

        for (LynxModule hub : hubs) {

            if (!hub.isParent()
                    && !String.valueOf(hub.getRevProductNumber())
                    .startsWith("1")) {

                return hub;
            }
        }

        throw new IllegalStateException("Expansion Hub not found");
    }

    @NonNull
    public static LynxModule getControlHub(HardwareMap hardwareMap) {

        return hardwareMap.getAll(LynxModule.class)
                .stream()
                .filter(LynxModule::isParent)
                .findFirst()
                .orElseThrow(
                        () -> new IllegalStateException(
                                "Control Hub not found"
                        )
                );
    }

    // ---------------- POSE ----------------

    public Pose getRawPose() {
        return follower.pose();
    }

    /**
     * Returns the predicted pose at projectile arrival time.
     */
    public Pose getPose() {

        if (sotm == null) {
            return getRawPose();
        }

        return sotm.getExpectedPose(getRawPose());
    }

    private double getDistance(Pose pose, Pose secondPose) {

        Pose minus = secondPose.minus(pose);

        return Math.hypot(minus.x(), minus.y());
    }

    public double getGoalDistance() {
        return getDistance(getPose(), getHive());
    }

    // ---------------- HIVE ----------------

    public void cycleHive() {
        hiveIndex = (hiveIndex == 0) ? 1 : 0;
    }

    public Pose getHive() {
        return hive[hiveIndex];
    }

    public void setHive() {

        if (Objects.equals(Constants.ALLIANCE_COLOR, "RED")) {

            hive = Constants.redHive;
            hiveIndex = 0;

        } else {

            hive = Constants.blueHive;
            hiveIndex = 1;
        }
    }

    public void initializedPoses() {

        setHive();

        follower.setPose(
                Objects.equals(Constants.ALLIANCE_COLOR, "RED")
                        ? Constants.redStartingPose
                        : Constants.blueStartingPose
        );
    }

    // ---------------- SOTM CONTROL ----------------

    public void enableShootingWhileMoving() {

        shootingWhileMovingEnabled = true;

        turret.disableAutoAlign();
    }

    public void disableShootingWhileMoving() {

        shootingWhileMovingEnabled = false;

        turret.clearShotOnMoveTarget();

        launcher.clearShotOnMoveTarget();

        turret.alignToHive();
    }

    public void toggleShootingWhileMoving() {

        if (shootingWhileMovingEnabled) {
            disableShootingWhileMoving();
        } else {
            enableShootingWhileMoving();
        }
    }

    /**
     * Calculates and applies the current SOTM shot solution.
     */
    private void updateShootingWhileMoving() {

        if (!shootingWhileMovingEnabled) {
            return;
        }

        // Always calculate from the raw pose.
        ShootingWhileMoving.ShotSolution solution =
                sotm.calculate(getRawPose());

        // Turret target and angular velocity
        turret.setShotOnMoveTarget(
                solution.turretAngle,
                solution.turretAngularVelocity
        );

        // Flywheel speed and hood position
        launcher.setShotOnMoveTarget(
                solution.wheelSpeed,
                solution.wheelAcceleration,
                Launcher.servoPos
        );

        // Telemetry
        telemetry.addData("SOTM Distance", solution.distance);
        telemetry.addData("SOTM TOF", solution.tof);

        telemetry.addData(
                "SOTM Turret Angle",
                solution.turretAngle
        );

        telemetry.addData(
                "SOTM Turret Velocity",
                solution.turretAngularVelocity
        );

        telemetry.addData(
                "SOTM Distance Rate",
                solution.distanceRate
        );

        telemetry.addData(
                "SOTM Wheel Speed",
                solution.wheelSpeed
        );

        telemetry.addData(
                "SOTM Wheel Acceleration",
                solution.wheelAcceleration
        );

        sotm.addTelemetry();
    }

    // ---------------- MAIN LOOP ----------------

    public void updateLoop() {

        // Clear hardware cache
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Update PedroPathing first
        follower.update();
        hiveIndex = (getPose().y() > 70.75)? 1:0;
        // Update velocity and acceleration estimates once per loop
        sotm.updateMotionEstimate();
        // Calculate and apply SOTM targets
        updateShootingWhileMoving();

        // Run subsystem periodic methods
        CommandScheduler.getInstance().run();

        // Drawing
        Drawing.drawRobot(follower.pose());

        Drawing.drawTarget(getHive());

        Drawing.update();
    }
}
