package org.firstinspires.ftc.teamcode.Globals;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;
import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
//import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public Follower follower = null;
    public MotorEx launchMotor;
    public  Telemetry telemetry;
    public Motor.Encoder launchEncoder;
    public Profiler profiler;
    public File file;
    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public Launcher launcher;
    //public Turret turret;
    public ServoEx hoodServo;

    public MotorEx frontLeft;
    public MotorEx frontRight;
    public MotorEx backLeft;
    public MotorEx backRight;


    public LynxModule ex;
    public List<LynxModule> hubs;
    HardwareMap.DeviceMapping<VoltageSensor> voltageSensor = null;
    public boolean Preload = false;
    public Limelight3A ll;

    public static Robot getInstance() {
        return instance;
    }

    public void init(HardwareMap hwMap){
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


        launchMotor = new MotorEx(hwMap, "Shooter")
                        .setCachingTolerance(0.01);
        launchMotor.setRunMode(Motor.RunMode.RawPower);
        launchMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launchEncoder = new Motor(hwMap, "Shooter").encoder;
        launchEncoder.setDirection(Motor.Direction.FORWARD);
        intakeMotor = new MotorEx(hwMap, "Intake")
                        .setCachingTolerance(0.01);
        transferMotor = new MotorEx(hwMap, "Transfer")
                        .setCachingTolerance(0.01);
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transferMotor.setInverted(true);
        transferMotor.setRunMode(Motor.RunMode.RawPower);
        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        hoodServo = new ServoEx(hwMap, "Hood").setCachingTolerance(0.001);
        //launcher = new Launcher();
        voltageSensor = hwMap.voltageSensor;
        // Drivetrain motors
        frontLeft = new MotorEx(hwMap, "fl")
                .setCachingTolerance(0.01);

        frontRight = new MotorEx(hwMap, "fr")
                .setCachingTolerance(0.01);

        backLeft = new MotorEx(hwMap, "bl")
                .setCachingTolerance(0.01);

        backRight = new MotorEx(hwMap, "br")
                .setCachingTolerance(0.01);

// Open-loop drive
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

// Brake when no power is applied
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

// Reverse the right side if required by your motor mounting
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
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
    @NonNull
    public static LynxModule getExpansionHub(List<LynxModule> ll) {
        ll.removeIf((hub1)->{return hub1.isParent() ||String.valueOf(hub1.getRevProductNumber()).startsWith("1");});
        return ll.get(0);
    }
    public void updateLoop() {
        ex.clearBulkCache();

        CommandScheduler.getInstance().run();
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
