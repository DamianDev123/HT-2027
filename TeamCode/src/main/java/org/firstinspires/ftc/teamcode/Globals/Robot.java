package org.firstinspires.ftc.teamcode.Globals;
//
//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;
import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public Follower follower = null;
    public MotorGroup launchMotors;
    public  Telemetry telemetry;
    public Motor.Encoder launchEncoder;
    public Profiler profiler;
    public File file;
    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public Launcher launcher;
    public Intake intake;
    public Turret turret;
    public ServoEx hoodServo;

    public ServoEx flapServo;
    public AprilTagProcessor aprilTag;
    public ServoEx turretServo1;
    public ServoEx turretServo2;
    public ServoEx intakeServo;
    public Motor.Encoder turretEncoder;


    public Servo led;
    public CRServo tilt;
    public ShootingWhileMoving shootingWhileMoving;
    public LynxModule ex;
    public List<LynxModule> hubs;
    HardwareMap.DeviceMapping<VoltageSensor> voltageSensor = null;
    public boolean Preload = false;
    public Limelight3A ll;

    public static Robot getInstance() {
        return instance;
    }

    public void init(HardwareMap hwMap, Follower _follower){
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();
        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
hubs = allHubs;
        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false) // Log EVERYTHING
                .build();
        ex = getControlHub(hwMap);
        ex.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = _follower;

        launchMotors = new MotorGroup(
                new MotorEx(hwMap, "Shooter")
                        .setCachingTolerance(0.01)
                        .setInverted(true),
                new MotorEx(hwMap, "shooterAssist")
                        .setCachingTolerance(0.01)

        );
        ll = hwMap.get(Limelight3A.class, "ll");
        launchMotors.setRunMode(Motor.RunMode.RawPower);
        launchMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launchEncoder = new Motor(hwMap, "Shooter").encoder;
        launchEncoder.setDirection(Motor.Direction.FORWARD);
        intakeServo = new ServoEx(hwMap, "intakeServo")
                .setCachingTolerance(0.01);
        tilt= new CRServo(hwMap, "tilt");
        intakeMotor = new MotorEx(hwMap, "Intake")
                        .setCachingTolerance(0.01);
        transferMotor = new MotorEx(hwMap, "Transfer")
                        .setCachingTolerance(0.01);
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transferMotor.setInverted(true);
        transferMotor.setRunMode(Motor.RunMode.RawPower);
        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        led = hwMap.get(Servo.class,"led");
        hoodServo = new ServoEx(hwMap, "Hood").setCachingTolerance(0.001);
        flapServo = new ServoEx(hwMap, "Flap").setCachingTolerance(0.001);

        turretServo1 = new ServoEx(hwMap, "turretServo1")
                        .setCachingTolerance(0.01);
        turretServo2 = new ServoEx(hwMap, "turretServo2")
                 .setCachingTolerance(0.01);

        turretEncoder = new Motor(hwMap, "Intake").encoder;
        intake = new Intake();

        launcher = new Launcher();
        turret = new Turret();
        shootingWhileMoving = new ShootingWhileMoving();
        voltageSensor = hwMap.voltageSensor;
    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public double getVoltage() {
        return 12/getBatteryVoltage();
    }
    public void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                Log.e("An error occurred", e.toString());
                Log.e(e.toString(), Arrays.toString(e.getStackTrace()));
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }
    @NonNull
    public static LynxModule getExpansionHub(List<LynxModule> ll) {
        ll.removeIf((hub1)->{return hub1.isParent() ||String.valueOf(hub1.getRevProductNumber()).startsWith("1");});
        return ll.get(0);
    }
    public void updateLoop() {
        ex.clearBulkCache();

        CommandScheduler.getInstance().run();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @NonNull
    public static LynxModule getControlHub(HardwareMap hardwareMap) {
        return hardwareMap.getAll(LynxModule.class)
                .stream()
                .filter(LynxModule::isParent)
                .findFirst()
                .orElseThrow(() -> new IllegalStateException("Control Hub not found"));
    }
}
