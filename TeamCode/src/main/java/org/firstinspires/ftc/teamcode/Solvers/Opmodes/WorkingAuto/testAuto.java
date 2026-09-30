package org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto;

import static org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.Globals.Constants.OpModeType;
import static org.firstinspires.ftc.teamcode.Globals.Constants.autoInitialized;
import static org.firstinspires.ftc.teamcode.Globals.Constants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.Globals.Constants.redGoalPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.WaitFull;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Turret;

import java.util.Objects;
import java.util.function.BooleanSupplier;

@Autonomous
public class testAuto extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();

    private Timer pathTimer, opmodeTimer;
    private ShootBalls shootBalls;

    public PathChain toLaunch, toLaunchBack;
    /* ------------------- Base Poses ------------------- */

    private Pose startPose   = new Pose(72, 72, Math.toRadians(90));
    private final Pose launchZone   = new Pose(72.000, 105.000);


    public void mirrorPaths() {
        startPose  = startPose.mirror();
        robot.GoalPose = blueGoalPose;
    }
    double mirrorAngle(double angle){
        return 180-angle;
    }

    /* ------------------- Path Building ------------------- */

    public void buildPaths() {

        toLaunch = follower.pathBuilder().addPath(
                new BezierLine(startPose, launchZone)
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        toLaunchBack = follower.pathBuilder().addPath(
                        new BezierLine(launchZone, startPose)
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

    }

    /* ------------------- Init ------------------- */

    @Override
    public void initialize() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Constants.OP_MODE_TYPE = OpModeType.AUTO;

        Limelight.autoRunning = true;
        robot.init(hardwareMap, follower);
        robot.telemetry = telemetry;
        autoInitialized = true;

        if (Objects.equals(ALLIANCE_COLOR, "BLUE")) {
            mirrorPaths();
            follower.setPose(startPose);
        } else {
            follower.setPose(startPose);
            robot.GoalPose = redGoalPose;
        }

        shootBalls = new ShootBalls();
        shootBalls.initialize();

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(() -> robot.intake.intake(true)),
                new FollowPathCommand(follower, toLaunch).withTimeout(250),
                new FollowPathCommand(follower, toLaunchBack)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        robot.updateLoop();
        robot.intake.runningAuto = true;
        telemetry.addData("Turr", robot.turret.forcedPos);

        telemetry.addData("pose", robot.follower.getPose());
        telemetry.update();
        // Intake.intakeDown();
    }
    public BooleanSupplier full = () -> Storage.full;

    @Override
    public void end() {
        robot.turret.setRunningAuto(true);
        robot.turret.shouldAim = false;

        robot.intake.runningAuto = false;
    }
}
