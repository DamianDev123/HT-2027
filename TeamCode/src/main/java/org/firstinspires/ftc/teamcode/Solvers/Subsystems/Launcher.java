package org.firstinspires.ftc.teamcode.Solvers.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
@Config
public class Launcher extends SubsystemBase {
    public static PIDCoefficients FLYWHEEL_PID_COEFFICIENTS = new PIDCoefficients(0.01, 0, 0); // Coefficients for ticks

    public static PIDCoefficients FLYWHEEL_PID_COEFFICIENTS2 = new PIDCoefficients(0.001, 0, 0); // Coefficients for ticks
    public static BasicFeedforwardParameters FLYWHEEL_FF_COEFFICIENTS = new BasicFeedforwardParameters
            (0.00045, 0,  0.32); // Coefficients for ticks
    private final Robot robot = Robot.getInstance();
    public static Double targetFlywheelVelocity = 0.0;
    public static Double MeasureVel = 1404.0;
    public static double vFeedback = 0.0;
    public static boolean isFlapOpen = false;
    public static boolean activeControl = false;

    public static boolean goFloat = false;
    public static double tolerances = 100;
    public static Boolean preload= false;
    public static Pose preloadPos = new Pose();
    public double errorAbs;
    public double error = 0;
    public boolean inTolerance = false;
    public boolean tolerable = false;
    public double distance = 0;

    public boolean doFlywheel = false;
    public boolean doHood = true;
    public boolean doPid = true;
    public static boolean tuning = false;
    public static double ff = 1;
    public static double closed = 0.42;
    public static double open =0.8;
    public static double timeout = 210;
    public boolean flapOpen = false;
    public double myTarget = 0;
    public static double errorTolerance = -50;
    public static double distanceOffset = 16;
    public static double distanceOffsetBlue = 14;

    public static double DISTANCE_OFFSET = -0.267;
    ElapsedTime timer = new ElapsedTime();


    public BasicFeedforwardParameters second =FLYWHEEL_FF_COEFFICIENTS;
    ControlSystem flywheelController = ControlSystem.builder()
            .velPid(FLYWHEEL_PID_COEFFICIENTS)
            .basicFF(second)

            .build();
    ControlSystem flywheelController2 = ControlSystem.builder()
            .velPid(FLYWHEEL_PID_COEFFICIENTS2)
            .basicFF(second)
            .build();
    public Launcher() {
        timer.startTime();
        timer.reset();

    }
    public void setFlap(Boolean onOff) {
        isFlapOpen = onOff;
    }

    public void setHood() {
        robot.hoodServo.set((0.9 - error * vFeedback)+0.01);
    }
    public void setFlywheel(double vel) {
        flywheelController.setGoal(new KineticState(0,Math.min(vel, Constants.LAUNCHER_MAX_VELOCITY)));
    }
    public double getFlywheelTarget() {
        return targetFlywheelVelocity;
    }
    public void stopFlywheel(){
        targetFlywheelVelocity = 0.0;
    }
    private void updateFlywheel() {
        second = new BasicFeedforwardParameters(0, FLYWHEEL_FF_COEFFICIENTS.kA, FLYWHEEL_FF_COEFFICIENTS.kS);
        second.kV = FLYWHEEL_FF_COEFFICIENTS.kV * robot.getVoltage();
        if (activeControl) {
            if (doPid || tuning) {
                goFloat = false;
                if(error<(errorTolerance) && !tuning)
                    robot.launchMotors.set(1);
                else
                    robot.launchMotors.set(flywheelController.calculate(new KineticState(0, robot.launchEncoder.getCorrectedVelocity())));
            }
            else {
                Float();
            }
        } else {
            if (getFlywheelTarget() == 0) {
                Float();
            } else {
                Float();
            }
        }

        setHood();

    }
    double distanceScalar(Double distance){
        if(Objects.equals(ALLIANCE_COLOR, "BLUE"))
            return distanceOffsetBlue;
        return distanceOffset;
    }
    @Override
    public void periodic() {

        robot.profiler.start("Launch Loop");
        setFlywheel(targetFlywheelVelocity);
        CalcLauncher();
        double max = 0.8;
        if(Storage.full)
            max = 1.0;
        if(!tuning){
            flywheelController2.setGoal(new KineticState(0,(robot.storage.contains || Storage.full) ? myTarget : myTarget/4 ));
            robot.launchMotors.set(Math.min(flywheelController2.calculate(new KineticState(0, robot.launchEncoder.getCorrectedVelocity())),max));
        }

        double vel = getTicksFromDist(distance);
        if(tuning)
            vel = MeasureVel;
        error = robot.launchEncoder.getCorrectedVelocity() - (vel);
        errorAbs = Math.abs(error);
        robot.flapServo.set(isFlapOpen ? open: closed);
        inTolerance = errorAbs < tolerances;
        Pose currentP = robot.CurrentPose;
        double dx = robot.PredictedGoalPose.getX() - currentP.getX();
        double dy =robot.PredictedGoalPose.getY() - currentP.getY();
        distance = Math.sqrt(dx * dx + dy * dy)+distanceScalar(distance);
        tolerable = errorAbs < 100;
        ;
        updateFlywheel();
        if(!isFlapOpen){
            timer.reset();}
        flapOpen = timer.milliseconds()>timeout;
        FtcDashboard.getInstance().getTelemetry().addData("Distance", distance);

        robot.profiler.end("Launch Loop");

    }
    public double getTicksFromDist(double x){
        return 7.23397*x+387.80466;
    }
    public double getFlywheelTicksFromVelocity(double v){
        return com.pedropathing.math.MathFunctions.clamp(5.89673*v+39.12313, 900,Constants.LAUNCHER_MAX_VELOCITY);
    }
    public void CalcLauncher(){

        double x = distance;
        myTarget =getTicksFromDist(x);
        if (doFlywheel || tuning) {
            if (tuning)
                targetFlywheelVelocity = MeasureVel;
            else {
                targetFlywheelVelocity= getTicksFromDist(x);

            }
            doPid = true;
        } else {
            doPid = false;
            Float();
        }

    }
    public void Float(){
        goFloat = true;
    }



}
