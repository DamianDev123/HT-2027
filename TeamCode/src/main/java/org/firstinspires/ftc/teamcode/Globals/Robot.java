package org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
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
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public static Follower follower = null;
    public MotorEx launchMotor;
    public  Telemetry telemetry;
    public Motor.Encoder launchEncoder;
    public Profiler profiler;
    public File file;
    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public Launcher launcher;
    public Storage storage;
    //public Turret turret;
    public ServoEx hoodServo;

    public MotorEx frontLeft;
    public MotorEx frontRight;
    public MotorEx backLeft;
    public ServoEx flap;
    public MotorEx backRight;
    public DigitalChannel breakBeam1;
    public Pose[] hive = new Pose[2];
    public int hiveIndex = 0;


    public LynxModule ex;
    public List<LynxModule> hubs;
    HardwareMap.DeviceMapping<VoltageSensor> voltageSensor = null;

    public static Robot getInstance() {
        return instance;
    }

    public void init(HardwareMap hwMap){
        if(follower == null){
            follower = Constants.create(hwMap);
            initializedPoses();
        }
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
        ex = getExpansionHub(allHubs);
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

        flap = new ServoEx(hwMap, "Flap").setCachingTolerance(0.001);
        hoodServo = new ServoEx(hwMap, "Hood").setCachingTolerance(0.001);
        breakBeam1 = hwMap.get(DigitalChannel.class, "transferBreakBeam");


        launcher = new Launcher();
        storage = new Storage();


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
    @NonNull
    public static LynxModule getExpansionHub(List<LynxModule> ll) {
        ll.removeIf((hub1)->{return hub1.isParent() ||String.valueOf(hub1.getRevProductNumber()).startsWith("1");});
        return ll.get(0);
    }
    public void updateLoop() {
        ex.clearBulkCache();

        CommandScheduler.getInstance().run();
        follower.update();
    }
    @NonNull
    public static LynxModule getControlHub(HardwareMap hardwareMap) {
        return hardwareMap.getAll(LynxModule.class)
                .stream()
                .filter(LynxModule::isParent)
                .findFirst()
                .orElseThrow(() -> new IllegalStateException("Control Hub not found"));
    }
    public void setHive(){
        if(Objects.equals(org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR, "RED")){
            hive = org.firstinspires.ftc.teamcode.Globals.Constants.redHive;
            hiveIndex = 1;
        }
        else{
            hive = org.firstinspires.ftc.teamcode.Globals.Constants.blueHive;
            hiveIndex=0;
        }

    }
    public void initializedPoses() {
        setHive();
        follower.setPose(Objects.equals(org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR, "RED")? org.firstinspires.ftc.teamcode.Globals.Constants.redStartingPose : org.firstinspires.ftc.teamcode.Globals.Constants.blueStartingPose);
    }

}
