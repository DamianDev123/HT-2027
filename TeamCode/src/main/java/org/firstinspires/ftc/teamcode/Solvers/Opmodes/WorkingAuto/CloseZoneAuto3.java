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

import org.firstinspires.ftc.teamcode.Solvers.Controllers.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.Constants;
import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Commands.ShootBalls;
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Autonomous
public class CloseZoneAuto3 extends CommandOpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();
    private Timer pathTimer, opmodeTimer;
    boolean shooting = false;
    private int pathState;
    ShootBalls shootBalls;

    public PathChain toLaunch,toRow1,toRow1Intake,toLaunchZoneRow1,toRow2,toRow2Intake,toGate,toLaunchZoneRow2,toRow3,toRow3Intake,toLaunchZoneRow3,toGatein,tolaunchGate,toGateInIn;

    private Pose startPose = new Pose(121.0,121,Math.toRadians(0));
    private Pose launchZone = new Pose(86.000, 98.000);
    private Pose launchZone2 = new Pose(96, 110);
    private Pose row1 = new Pose(130.000, 36.500);
    private Pose row1End = new Pose(120.500, 35.500);
    private Pose row2 = new Pose(130.000, 59.500);
    private Pose row2End = new Pose(128.500, 59.500);
    private Pose gate = new Pose(110, 70.640);
    private Pose row3 = new Pose(120.000, 83.500);
    private Pose row3End = new Pose(128.500, 83.500);
    private Pose park =  new Pose(96.000, 20.000);


    private final List<Pose> controlPoints = new ArrayList<>(List.of(
            new Pose(93.267, 83.700),
            new Pose(91.5, 84),
            new Pose(112.473, 67.707),
            new Pose(93.153, 60.137),
            new Pose(110.303, 58.537),
            new Pose(90.447, 31.963),
            new Pose(85, 31)
    ));
    private double intakeHeading = 0.0;
    public void mirrorPaths(){
        startPose = startPose.mirror();
        row1 = row1.mirror();
        row1End = row1End.mirror();
        row2 = row2.mirror();
        row2End = row2End.mirror();
        gate = gate.mirror();
        row3 = row3.mirror();
        row3End = row3End.mirror();
        park = park.mirror();
        launchZone = launchZone.mirror();

        for (int i = 0; i < controlPoints.size(); i++) {
            controlPoints.set(i, controlPoints.get(i).mirror());
        }

        intakeHeading = 180;
        robot.GoalPose = blueGoalPose;
    }

    public void buildPaths() {
        toLaunch = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,

                                launchZone
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), intakeHeading)
                .build();
        toRow3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchZone,
                                controlPoints.get(0),
                                row3
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();
        toRow3Intake = follower.pathBuilder().addPath(
                        new BezierLine(
                                row3,
                                row3End
                        )
                ).setTangentHeadingInterpolation()

                .build();
        toLaunchZoneRow3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row3,
                                controlPoints.get(2),
                                launchZone
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();
        toRow2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchZone,
                                controlPoints.get(3),
                                row2
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();
        toRow2Intake = follower.pathBuilder().addPath(
                        new BezierLine(
                               row2,

                               row2End
                        )
                ).setTangentHeadingInterpolation()

                .build();
        toLaunchZoneRow2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row2End,
                                controlPoints.get(4),
                                launchZone
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();
        toRow1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                               launchZone,
                                controlPoints.get(5),
                                row1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
                .build();
        toRow1Intake = follower.pathBuilder().addPath(
                        new BezierLine(
                                row1,

                                row1End
                        )
                ).setTangentHeadingInterpolation()

                .build();
        toLaunchZoneRow1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                row1End,
                                controlPoints.get(6),
                               launchZone
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))

                .build();
        toGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.000, 98.000),
                                new Pose(102.949, 62.543),
                                new Pose(122.000, 70.246)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();


        toGatein = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(122.000, 70.246),
                                new Pose(128.472, 50.625),
                                new Pose(136.048, 49)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                .build();
        toGateInIn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(136.048, 49),

                                new Pose(136.131, 60)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(65))

                .build();


        tolaunchGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(136.131, 60),
                                new Pose(104.239, 58.675),
                                new Pose(86.000, 98.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                .build();

    }


    @Override
    public void initialize() {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Constants.OP_MODE_TYPE = OpModeType.AUTO;
        robot.init(hardwareMap,follower);
        autoInitialized = true;

        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            follower.setPose(startPose.mirror());
        }else {
            follower.setPose(startPose);
        }

        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            robot.GoalPose = blueGoalPose;
        }else {
            robot.GoalPose = redGoalPose;
        }
        follower.update();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shootBalls = new ShootBalls();
        if(Objects.equals(ALLIANCE_COLOR, "BLUE")){
            mirrorPaths();
        }
        robot.turret.setRunningAuto(true);
        shootBalls.initialize();
        buildPaths();
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(()-> autoInitialized = true),
                new InstantCommand(()->robot.turret.shouldAim = true),
                new InstantCommand(()-> robot.launcher.setFlap(true)),
                new FollowPathCommand(follower,toLaunch, true).alongWith(new ShootBalls()),



                new InstantCommand(()-> robot.intake.intake(true)),
                new FollowPathCommand(follower,toRow2).interruptOn(robot.intake.supplier),
                new WaitUntilCommand(robot.intake.supplier).withTimeout(1000),
                new FollowPathCommand(follower,toLaunchZoneRow2, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new ShootBalls(),



                new FollowPathCommand(follower,toGate).withTimeout(3000),
                new InstantCommand(()-> robot.intake.intake(true)),
                new FollowPathCommand(follower,toGatein).withTimeout(2000).interruptOn(robot.intake.supplier),
                new FollowPathCommand(follower,toGateInIn).withTimeout(300).interruptOn(robot.intake.supplier),
                new WaitUntilCommand(robot.intake.supplier).withTimeout(1000),
                new FollowPathCommand(follower,tolaunchGate,true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new ShootBalls(),


                new InstantCommand(()-> robot.turret.toggle =false),
                new InstantCommand(()-> robot.intake.intake(true)),
                new FollowPathCommand(follower,toRow3).interruptOn(robot.intake.supplier),
                new WaitUntilCommand(robot.intake.supplier).withTimeout(1000),
                new FollowPathCommand(follower,toLaunchZoneRow3, true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true))),
                new ShootBalls(),

                new InstantCommand(()-> robot.launcher.setFlap(false)),
                new FollowPathCommand(follower,toGate).withTimeout(3000),
                new InstantCommand(()-> robot.intake.intake(true)),
                new FollowPathCommand(follower,toGatein).withTimeout(2000).interruptOn(robot.intake.supplier),
                new FollowPathCommand(follower,toGateInIn).withTimeout(300).interruptOn(robot.intake.supplier),
                new WaitUntilCommand(robot.intake.supplier).withTimeout(1000),
                new FollowPathCommand(follower,tolaunchGate,true).halfWay(new InstantCommand(()-> robot.intake.intake(false)),new InstantCommand(()-> robot.launcher.setFlap(true)))
                ,
                new ShootBalls()

        );

        schedule(
                autonomousSequence
        );

    }
    @Override
    public void run(){
        super.run();
        robot.updateLoop();
    }
}
