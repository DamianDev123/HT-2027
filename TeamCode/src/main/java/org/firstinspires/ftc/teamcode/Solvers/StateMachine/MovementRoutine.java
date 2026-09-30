package org.firstinspires.ftc.teamcode.Solvers.StateMachine;

import static org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.CloseZoneAuto5.cp;
import static org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.CloseZoneAuto5.gatein;

import static java.lang.Math.atan2;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.CloseZoneAuto5;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.State;
import org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions.StateMachine;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;

@Config
public class MovementRoutine implements StateMachine {
    Follower follower;
    public Gamepad gamepad;
    ActionRoutine actionRoutine;
    private Supplier<PathChain> toGate;
    private Supplier<PathChain> gateIn;
    private Supplier<PathChain> toLaunch;
    private final Pose launch = new Pose(76.203, 79.630);
    public static PIDFCoefficients headingCoefficients = new PIDFCoefficients(0.4,0,0.001,0.02);
    public static PIDFController headingController = new PIDFController(headingCoefficients);
    MovementRoutine(Follower follower, Gamepad gamepad, ActionRoutine actionRoutine){
        this.follower = follower;
        this.gamepad = gamepad;
        this.actionRoutine = actionRoutine;


        toGate = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, CloseZoneAuto5.gate)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(30), 0.8))
                .build();
        gateIn = () -> follower.pathBuilder().addPath(
                        new BezierCurve(
                                follower::getPose,
                                cp,
                                gatein
                        )
                ).setLinearHeadingInterpolation(
                        Math.toRadians(30),
                        Math.toRadians(30))
                .build();
        toLaunch = () -> follower.pathBuilder().addPath(
                        new BezierLine(
                                follower::getPose,
                                launch
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        initStateMachine();

    }
    @State(first = true)
    public void driving(){
        if(justEntered())
            follower.startTeleopDrive();
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true
        );

    }
    @State()
    public void aligning(){
        Pose currentPose = robot.CurrentPose;
        double currentGoal = atan2((robot.PredictedGoalPose.getY() - currentPose.getY()), (robot.PredictedGoalPose.getX() - currentPose.getX()));
        if(!Turret.backWards)
            currentGoal = Math.toRadians(180-Math.toDegrees(currentGoal));
        FtcDashboard.getInstance().getTelemetry().addData("angle",currentGoal);
        if(justEntered())
            follower.startTeleopDrive();

        headingController.setTargetPosition(currentGoal);
        headingController.updatePosition(robot.CurrentPose.getHeading());
        double r = headingController.run();
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true
        );
        if(!robot.storage.contains)
            nextState();

    }
    public void startGateSequence(){
        nextState("toGateM");
    }
    @State()
    public void toGateM(){
        follower.followPath(toGate.get());
        if(follower.getCurrentTValue()>0.9)
            nextState("toGateInM");
    }
    @State()
    public void toGateInM(){
        actionRoutine.nextState("Intaking");
        follower.followPath(gateIn.get());
        if(follower.getCurrentTValue()>0.9)
            nextState();
    }
    @State()
    public void toLaunchM(){
        follower.followPath(toLaunch.get());
        if(follower.getCurrentTValue()>0.9)
            nextState();
    }
}
