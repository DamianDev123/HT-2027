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
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Solvers.Controllers.WaitFull;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Filter;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;

import java.util.Objects;
import java.util.function.BooleanSupplier;

@Autonomous
public class CloseZoneAuto4 extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();

    private Timer pathTimer, opmodeTimer;
    private ShootBalls shootBalls;

    public PathChain toLaunch,
            toRow1, toRow1Intake, toLaunchZoneRow1,
            toRow2, toRow2Intake, toLaunchZoneRow2,
            toRow3, toRow3Intake, toLaunchZoneRow3,
            toGate, toGateIn, tolaunchGate, GoIn,
            GoBack, goBacky,goBecky
            ;

    /* ------------------- Base Poses ------------------- */

    private Pose startPose   = new Pose(121.0, 121.0, Math.toRadians(0));
    private Pose launchZone  = new Pose(90, 98.000);
    private Pose row1        = new Pose(139.000, 36.500);
    private Pose row1End     = new Pose(120.500, 35.500);
    private Pose row2        = new Pose(139.000, 55.500);
    private Pose row2End     = new Pose(128.500, 59.500);
    private Pose row3        = new Pose(127.000, 83.500);
    private Pose row3End     = new Pose(128.500, 83.500);
    private Pose gate        = new Pose(130, 69);


    /* ------------------- Control Points ------------------- */

    private Pose cpRow3Out = new Pose(93.267, 83.700);
    private Pose cpGateMid = new Pose(112.473, 67.707);

    private Pose cpRow2Out = new Pose(86, 54);
    private Pose cpRow2In  = new Pose(110.303, 58.537);

    private Pose cpRow1Out = new Pose(86.26928895612707, 25);
    private Pose cpRow1In  = new Pose(85.000, 31.000);

    private Pose becky = new Pose(124, 14);
    private Pose backy = new Pose(132.179, 10.419);
    private Pose cp = new Pose(125.40998487140696, 50.959909228441724);
    private Pose posy = new Pose(128.5, 57);
    private Pose ss = new Pose(96.496, 52.888);
    private Pose helpMe = new Pose(93, 83.365);
    private Pose omg = new Pose(100.370, 50.507);
    private double intakeHeading = 0.0;
    private double beckyHead = -60.0;
    private double shissh = 0.0;
    private double mijo = 20.0;
    

    /* ------------------- Alliance Mirroring ------------------- */

    public void mirrorPaths() {
        startPose  = startPose.mirror();
        launchZone = launchZone.mirror();
        row1       = row1.mirror();
        row1End    = row1End.mirror();
        row2       = row2.mirror();
        row2End    = row2End.mirror();
        row3       = row3.mirror();
        row3End    = row3End.mirror();
        gate       = gate.mirror();
        becky = becky.mirror();
        backy = backy.mirror();
        cp = cp.mirror();
        posy = posy.mirror();
        ss = ss.mirror();
        helpMe = helpMe.mirror();
        omg = omg.mirror();

        cpRow3Out = cpRow3Out.mirror();
        cpGateMid = cpGateMid.mirror();
        cpRow2Out = cpRow2Out.mirror();
        cpRow2In  = cpRow2In.mirror();
        cpRow1Out = cpRow1Out.mirror();
        cpRow1In  = cpRow1In.mirror();

        intakeHeading = 180;
        beckyHead = mirrorAngle(beckyHead);
        shissh = mirrorAngle(shissh);
        mijo = mirrorAngle(mijo);

        robot.GoalPose = blueGoalPose;
    }
    double mirrorAngle(double angle){
        return 180-angle;
    }

    /* ------------------- Path Building ------------------- */

    public void buildPaths() {

        toLaunch = follower.pathBuilder().addPath(
                new BezierLine(startPose, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow3 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone, cpRow3Out, row3)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow3Intake = follower.pathBuilder().addPath(
                new BezierLine(row3, row3End)
        ).setTangentHeadingInterpolation().build();

        toLaunchZoneRow3 = follower.pathBuilder().addPath(
                new BezierCurve(row3, cpGateMid, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow2 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone, cpRow2Out, row2)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow2Intake = follower.pathBuilder().addPath(
                new BezierLine(row2, row2End)
        ).setTangentHeadingInterpolation().build();

        toLaunchZoneRow2 = follower.pathBuilder().addPath(
                new BezierCurve(row2End, cpRow2In, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toRow1 = follower.pathBuilder().addPath(
                new BezierCurve(launchZone, cpRow1Out, row1)
        ).setTangentHeadingInterpolation(
        ).build();

        toRow1Intake = follower.pathBuilder().addPath(
                new BezierLine(row1, row1End)
        ).setTangentHeadingInterpolation().build();

        toLaunchZoneRow1 = follower.pathBuilder().addPath(
                new BezierCurve(row1End, cpRow1In, launchZone)
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        toGate = follower.pathBuilder().addPath(
                new BezierCurve(
                        launchZone,
                        omg,
                        gate
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();
        toGateIn = follower.pathBuilder().addPath(
                new BezierCurve(
                        gate,
                         cp,
                        posy
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();

        tolaunchGate = follower.pathBuilder().addPath(
                new BezierCurve(
                       gate,
                        ss,
                        helpMe
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();
        GoIn =  follower.pathBuilder().addPath(
                new BezierLine(
                        launchZone,
                        backy
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(beckyHead),
                Math.toRadians(beckyHead)

        ).build();
        goBacky =  follower.pathBuilder().addPath(
                new BezierLine(
                        backy,
                        becky
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(beckyHead),
                Math.toRadians(beckyHead)).build();
        goBecky =  follower.pathBuilder().addPath(
                new BezierLine(
                       becky,
                        backy
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(beckyHead),
                Math.toRadians(beckyHead)).build();
        GoBack=  follower.pathBuilder().addPath(
                        new BezierLine(
                                backy,
                                launchZone
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
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
                new FollowPathCommand(follower, toLaunch, true),
                new ShootBalls(),


                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new InstantCommand(() -> robot.intake.intake(true)),

                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, toRow2).interruptOn(robot.intake.supplier),
                new WaitFull(),
                new InstantCommand(Intake::intakeDown),

                new FollowPathCommand(follower, toLaunchZoneRow2)
                        .halfWay(
                                new InstantCommand(() -> robot.intake.intake(false))
                        ),
                new WaitCommand(200),
                new ShootBalls(),

                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, toGate).withTimeout(3000),
                new FollowPathCommand(follower, tolaunchGate),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower,toRow3).halfWay(
                        new InstantCommand(() -> robot.intake.intake(true))).interruptOn(robot.intake.supplier),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new WaitFull(),
                new FollowPathCommand(follower,toLaunchZoneRow3, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new WaitCommand(200),
                new ShootBalls(),
                new InstantCommand(() -> robot.intake.intake(true)),
                new FollowPathCommand(follower,toRow1).interruptOn(robot.intake.supplier),
                new WaitFull(),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower,toLaunchZoneRow1, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),

                new ShootBalls(),
                new TurnToCommand(follower,Math.toRadians(beckyHead)).withTimeout(100),
                new FollowPathCommand(follower, GoIn).halfWay(new InstantCommand(()->robot.intake.intake(true))).withTimeout(3600).interruptOn(robot.intake.supplier),
                new FollowPathCommand(follower, goBacky).interruptOn(robot.intake.supplier).withTimeout(600),
                new FollowPathCommand(follower, goBecky).interruptOn(robot.intake.supplier).withTimeout(600),
                new WaitFull(),
                new FollowPathCommand(follower, GoBack),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new TurnToCommand(follower,Math.toRadians(mijo)),
                new WaitCommand(200),
                new SequentialCommandGroup(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),
                new FollowPathCommand(follower, GoIn).halfWay(new InstantCommand(()->robot.intake.intake(true))).withTimeout(3600).interruptOn(robot.intake.supplier),
                new FollowPathCommand(follower, goBacky).interruptOn(robot.intake.supplier).withTimeout(600),
                new FollowPathCommand(follower, goBecky).interruptOn(robot.intake.supplier).withTimeout(600),
                new WaitFull(),
                new FollowPathCommand(follower, GoBack),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new TurnToCommand(follower,Math.toRadians(mijo)),
                new WaitCommand(200),
                new SequentialCommandGroup(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))).alongWith(new SequentialCommandGroup(new WaitUntilCommand(nearEnd),new ShootBalls())),
                new FollowPathCommand(follower, GoIn).halfWay(new InstantCommand(()->robot.intake.intake(true))).withTimeout(2000)


        );

        schedule(autonomousSequence);
    }
    public BooleanSupplier nearEnd = ()-> follower.getCurrentTValue()>0.8;

    @Override
    public void run() {
        super.run();
        robot.updateLoop();
        follower.setMaxPower(1);
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
