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

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;

import java.util.Objects;
import java.util.function.BooleanSupplier;

@Autonomous
public class FarZoneAuto extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();

    private Timer pathTimer, opmodeTimer;
    private ShootBalls shootBalls;

    public PathChain toIntakeHuman,toSideHuman, toLaunchHuman, toIntakeRow1, toLaunchRow1;

    /* ------------------- Base Poses ------------------- */
    public Pose humanIntake = new Pose(136, 20.000);
    public Pose humanSide = new Pose(136.000, 9.000);
    public Pose row1 = new Pose(132.000, 36.500);
    private Pose cpRow1I = new Pose(105.000, 35.500);

    private Pose startPose = new
            Pose(96.0,8.0,Math.toRadians(90.0));
    private Pose launchZone  = new Pose(96, 8);
    private double intakeHeading = 0.0;

    /* ------------------- Alliance Mirroring ------------------- */

    public void mirrorPaths() {
        startPose  = startPose.mirror();
        launchZone = launchZone.mirror();
        humanIntake = humanIntake.mirror();
        humanSide = humanSide.mirror();
        cpRow1I = cpRow1I.mirror();
        row1 = row1.mirror();

        intakeHeading = 180;
        robot.GoalPose = blueGoalPose;
    }

    /* ------------------- Path Building ------------------- */

    public void buildPaths() {
        toIntakeHuman = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,

                                humanIntake
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
                .build();
        toSideHuman = follower.pathBuilder().addPath(
                        new BezierLine(
                                humanIntake,

                                humanSide
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
                .build();
        toLaunchHuman = follower.pathBuilder().addPath(
                        new BezierLine(
                                humanSide,

                                launchZone
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
                .build();
        toIntakeRow1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchZone,
                                cpRow1I,
                                row1
                        )
                ).setTangentHeadingInterpolation()

                .build();
        toLaunchRow1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row1,
                                cpRow1I,
                                launchZone
                        )
                ).setTangentHeadingInterpolation()

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

        robot.turret.setRunningAuto(true);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(() -> robot.turret.shouldAim = true),
                new InstantCommand(() -> robot.launcher.setFlap(true)),
                new ShootBalls(1000),
                new InstantCommand(()->robot.intake.intake()),
                new FollowPathCommand(follower,toIntakeHuman),
                new FollowPathCommand(follower, toSideHuman),
                new FollowPathCommand(follower, toLaunchHuman),
                new ShootBalls(1000),
                new InstantCommand(()->robot.intake.intake()),
                new FollowPathCommand(follower, toIntakeRow1),
                new FollowPathCommand(follower, toLaunchHuman),
                new ShootBalls(1000),
                new WaitCommand(3000),
                new InstantCommand(this::requestOpModeStop)


        );

        schedule(autonomousSequence);
    }
    public BooleanSupplier nearEnd = ()-> follower.getCurrentTValue()>0.8;

    @Override
    public void run() {
        super.run();
        robot.updateLoop();

        robot.intake.runningAuto = true;
        // Intake.intakeDown();
    }

    @Override
    public void end() {
        robot.turret.setRunningAuto(true);
        robot.turret.shouldAim = false;

        robot.intake.runningAuto = false;
    }
}
