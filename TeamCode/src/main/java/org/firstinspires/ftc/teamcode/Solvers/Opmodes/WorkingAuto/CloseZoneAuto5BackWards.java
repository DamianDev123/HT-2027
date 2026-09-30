package org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto;

import static org.firstinspires.ftc.teamcode.Globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.Globals.Constants.OpModeType;
import static org.firstinspires.ftc.teamcode.Globals.Constants.autoInitialized;
import static org.firstinspires.ftc.teamcode.Globals.Constants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.Globals.Constants.redGoalPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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

import java.util.ArrayList;
import java.util.Objects;
import java.util.function.BooleanSupplier;

@Autonomous
public class CloseZoneAuto5BackWards extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();

    private Timer pathTimer, opmodeTimer;
    private ShootBalls shootBalls;

    public PathChain toLaunch,
            toRow1, toRow1Intake, toLaunchZoneRow1,
            toRow2, toRow2Intake, toLaunchZoneRow2,
            toRow3, toRow3Intake, toLaunchZoneRow3,
            toGate, toGateIn, tolaunchGate, gateIn
            ;

    /* ------------------- Base Poses ------------------- */

    private Pose startPose   = new Pose(118, 130.5, Math.toRadians(38));
    public static Pose launchZone  = new Pose(87, 89);
    private Pose row1        = new Pose(139.000, 36.500);
    private Pose row1End     = new Pose(120.500, 35.500);
    private Pose row2        = new Pose(134.000, 56);
    private Pose row2End     = new Pose(128.500, 59.500);
    private Pose row3        = new Pose(130.000, 87);
    private Pose row3End     = new Pose(128.500, 83.500);
    public static Pose gate        = new Pose(130 , 64);
    public static Pose gatein        = new Pose(130 , 56);


    /* ------------------- Control Points ------------------- */

    private Pose cpRow3Out = new Pose(93.267, 83.700);
    private Pose cpGateMid = new Pose(112.473, 67.707);

    private Pose cpRow2Out = new Pose(86, 54);
    private Pose cpRow2In  = new Pose(110.303, 58.537);

    private Pose cpRow1Out = new Pose(86.26928895612707, 25);
    private Pose cpRow1In  = new Pose(85.000, 31.000);

    private Pose becky = new Pose(124, 14);
    private Pose backy = new Pose(132.179, 10.419);
    public static Pose cp = new Pose(123, 58.678517397882004);
    private Pose posy = new Pose(124, 57);
    private Pose pos = new Pose(80, 70);
    public static Pose ss = new Pose(96.496, 52.888);
    private Pose helpMe = new Pose(93, 83.365);
    private Pose omg = new Pose(100.370, 50.507);
    private double intakeHeading = 0.0;
    private double beckyHead = -60.0;
    private double shissh = 0.0;
    private double mijo = 20.0;
    private double Rece = 30.0;
    

    /* ------------------- Alliance Mirroring ------------------- */

    public void mirrorPaths() {
        PathChain MainChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.381, 121.648),
                                new Pose(82.366, 84.478)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(225))
                .addPath(
                        new BezierCurve(
                                new Pose(82.366, 84.478),
                                new Pose(57.908, 52.876),
                                new Pose(126.104, 59.291)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        PathChain MainChain2 = MainChain;
        startPose  = startPose.mirror();
        launchZone = launchZone.mirror();
        row1       = row1.mirror();
        row1End    = row1End.mirror();
        row2       = row2.mirror();
        row2End    = row2End.mirror();
        row3       = row3.mirror();
        row3End    = row3End.mirror();
        gate       = gate.mirror();
        gatein = gatein.mirror();
        becky = becky.mirror();
        backy = backy.mirror();
        cp = cp.mirror();
        posy = posy.mirror();
        ss = ss.mirror();
        helpMe = helpMe.mirror();
        omg = omg.mirror();
        pos = pos.mirror();

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
        Rece = mirrorAngle(Rece);

        robot.GoalPose = blueGoalPose;
    }
    PathChain mirrorPathChain(PathChain pathChain){
        Path[] paths = new Path[pathChain.size()];
        for (int i = 0; i < pathChain.size(); i++) {
            Path path = pathChain.getPath(i);
            ArrayList<Pose> poses = path.getControlPoints();
            poses.replaceAll(Pose::mirror);
            double startAngle = mirrorAngleR(path.getPose(0).getHeading());
            double endAngle = mirrorAngleR(path.getPose(1).getHeading());
            Curve curve;
            if(path.pathType().equals("line")){
                curve = new BezierLine(
                        poses.get(0),
                        poses.get(1)
                );
            }else {
                curve = new BezierCurve(
                        poses
                );
            }
            Path newPath = new Path(curve);
            newPath.setLinearHeadingInterpolation(startAngle,endAngle);
            newPath.setHeadingInterpolation(path.getHeadingInterpolator());
            paths[i] = newPath;

        }

        return follower.pathBuilder().addPaths(paths).build();
    }
    double mirrorAngle(double angle){
        return 180-angle;
    }
    double mirrorAngleR(double angle){
        return 180-angle;
    }

    /* ------------------- Path Building ------------------- */

    public void buildPaths() {

        toLaunch = follower.pathBuilder().addPath(
                new BezierLine(startPose, launchZone)
        ).setTangentHeadingInterpolation().setReversed()
                .build();

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
                        pos,
                        gate
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(38),
                Math.toRadians(intakeHeading)
        ).build();
        gateIn = follower.pathBuilder().addPath(
                new BezierCurve(
                        gate,
                        cp,
                        gatein
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(Rece)
        ).build();
        toGateIn = follower.pathBuilder().addPath(
                new BezierCurve(
                        gate,
                        cp,
                        posy
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(Rece),
                Math.toRadians(Rece)
        ).build();

        tolaunchGate = follower.pathBuilder().addPath(
                new BezierCurve(
                       gate,
                        ss,
                        launchZone
                )
        ).setLinearHeadingInterpolation(
                Math.toRadians(intakeHeading),
                Math.toRadians(intakeHeading)
        ).build();
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
        robot.turret.forcedPos = launchZone;
        robot.turret.overrideTurret(launchZone);

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(() -> robot.turret.shouldAim = true),
                new InstantCommand(()->robot.turret.setOverride(true)),
                new InstantCommand(() -> robot.launcher.setFlap(true)),
                new FollowPathCommand(follower, toLaunch),
                new WaitCommand(200),
                new ShootBalls(),

                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new InstantCommand(() -> robot.intake.intake(true)),

                new InstantCommand(() -> robot.launcher.setFlap(false)),

                new FollowPathCommand(follower, toRow2).interruptOn(robot.intake.supplier),
                new InstantCommand(Intake::intakeDown),
                new InstantCommand(()-> Turret.offsetR=14),

                new WaitCommand(200),
                new FollowPathCommand(follower, toLaunchZoneRow2),
                new InstantCommand(() -> robot.intake.intake(false)),
                new WaitCommand(200),
                new ShootBalls(),

                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, toGate).withTimeout(3000),
                new WaitCommand(250),
                new FollowPathCommand(follower, gateIn).withTimeout(400),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull().interruptOn(full),
                new WaitFull().interruptOn(full),
                new InstantCommand(() -> robot.intake.intake(false)),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, tolaunchGate),
                new ShootBalls(),
                new InstantCommand(() -> robot.launcher.setFlap(false)),


                new FollowPathCommand(follower,toRow3).halfWay(
                        new InstantCommand(() -> robot.intake.intake(true))).interruptOn(robot.intake.supplier),
                new InstantCommand(() -> robot.launcher.setFlap(false)),

                new InstantCommand(() -> robot.intake.intake(false)),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower,toLaunchZoneRow3, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new WaitCommand(200),
                new ShootBalls(),


                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, toGate).withTimeout(3000),
                new WaitCommand(250),
                new FollowPathCommand(follower, gateIn).withTimeout(400),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull().interruptOn(full),
                new WaitFull().interruptOn(full),
                new InstantCommand(() -> robot.intake.intake(false)),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, tolaunchGate),
                new ShootBalls(),

                new InstantCommand(() -> robot.launcher.setFlap(false)),

                new FollowPathCommand(follower, toGate).withTimeout(3000),
                new WaitCommand(250),
                new FollowPathCommand(follower, gateIn).withTimeout(400),
                new InstantCommand(() -> robot.intake.intake(true)),
                new WaitFull().interruptOn(full),
                new WaitFull().interruptOn(full),
                new InstantCommand(() -> robot.intake.intake(false)),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower, tolaunchGate),
                new ShootBalls(),

                new InstantCommand(() -> robot.intake.intake(true)),
                new FollowPathCommand(follower,toRow1).interruptOn(robot.intake.supplier),
                new WaitFull(),
                new InstantCommand(() -> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower,toLaunchZoneRow1, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),

                new ShootBalls(),
                new TurnToCommand(follower,Math.toRadians(beckyHead)).withTimeout(100)
        );

        schedule(autonomousSequence);
    }
    public BooleanSupplier nearEnd = ()-> follower.getCurrentTValue()>0.8;

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
