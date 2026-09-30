package org.firstinspires.ftc.teamcode.Solvers.StateMachine;

import static org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.Globals.Constants.autoInitialized;
import static org.firstinspires.ftc.teamcode.Globals.Constants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.Globals.Constants.redGoalPose;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;

import java.util.Objects;

@TeleOp
@Config
public class StateMachineTeleop extends CommandOpMode{

    Follower follower;

    public static boolean wantsToShoot = false;
    private final Robot robot = Robot.getInstance();
    public GamepadEx operator;
    public ActionRoutine actionRoutine;
    public MovementRoutine movementRoutine;

    private boolean prevTrigger;
    private boolean intakeActive;

    Runnable interruptAllStates = () -> {
        movementRoutine.nextState();
        actionRoutine.nextState();
    };
    @Override
    public void initialize() {
        operator = new GamepadEx(gamepad2);
        Limelight.Companion.createFollower(hardwareMap);



        setGoalPose();
        initializeFollowerAndRobot();
        lightUp();
        follower.update();

        robot.turret.inAuto = false;
        robot.intake.runningAuto = false;

        actionRoutine = new ActionRoutine();
        movementRoutine = new MovementRoutine(robot.follower,gamepad2,actionRoutine);
    }
    private void handleIntake() {
        boolean trigger = operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;

        if (trigger && !prevTrigger) {
            actionRoutine.nextState(wantsToShoot ? "Shooting" : "Intaking");
            intakeActive = !wantsToShoot;
            if(wantsToShoot)
                wantsToShoot = false;
        }
        if (!trigger && prevTrigger && intakeActive && actionRoutine.getCurrentStateName().equals("Intaking")) {
            actionRoutine.nextState("Reset");
            intakeActive = false;
        }

        prevTrigger = trigger;
    }
    boolean temp = false;
    ElapsedTime timer = new ElapsedTime();


    @Override public void run(){
        setGoalPose();
        Launcher.activeControl = true;
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this::wantShoot);
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Intake::intakeUp);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(movementRoutine::startGateSequence);
        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(interruptAllStates);

        handleIntake();
        robot.updateLoop();
        actionRoutine.update();
        if(operator.gamepad.dpad_left)
            Turret.offsetR+=0.3;
        if(operator.gamepad.dpad_right)
            Turret.offsetR-=0.3;
        movementRoutine.update();
    }
    void wantShoot(){
        wantsToShoot = true;
        movementRoutine.nextState("aligning");
    }
    void initializeFollowerAndRobot(){
        if(!autoInitialized) {
            follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        }else {
            follower = robot.follower;
        }
        robot.init(hardwareMap,follower);
        robot.telemetry = telemetry;
        if(!autoInitialized){
            if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
                follower.setPose(robot.poses.getStartFromFar().mirror());
            }else {
                follower.setPose(robot.poses.getStartFromFar());
            }
        }
    }
    void setGoalPose(){

            robot.GoalPose = redGoalPose;


    }
    void lightUp(){
        for(LynxModule hub : robot.hubs){
            hub.setConstant(Objects.equals(ALLIANCE_COLOR, "BLUE") ? Color.BLUE : Color.RED);
        }
    }


}
