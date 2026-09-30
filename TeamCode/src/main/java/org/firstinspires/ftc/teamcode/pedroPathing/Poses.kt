package org.firstinspires.ftc.teamcode.pedroPathing

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.Globals.Constants
import org.firstinspires.ftc.teamcode.Globals.Robot
import kotlin.math.PI


class Poses {

    val robot: Robot = Robot.getInstance();
    var startFromClose = Pose(121.0, 121.0, Math.toRadians(0.0));
    var startFromFar = Pose(96.0,8.0,Math.toRadians(90.0));
    var start = Pose(89.0, 8.0, PI / 2)
    var launchZone = Pose(96.0, 8.0, PI / 2)
    var row1 = Pose(100.0, 35.5, 0.0)// Row closest to the front of the field
    var row1End = Pose(129.0, 36.0, 0.0) // End of row 1 (e.g. where we stop intake)
    var row2 = Pose(110.0, 59.5, 0.0) // Row seconds closest
    var row2End = Pose(129.0, 59.5, 0.0)
    var row3 = Pose(110.0, 83.5, 0.0) // Row closest to the classifier
    var row3End = Pose(129.0, 83.5, 0.0)
    var ramp = Pose(130.5,70.0, 0.0)
    var fromRamp = Pose(130.5,70.0, 0.0)
    var row1Intake: PathChain? = null;
    var row2Intake: PathChain? = null;
    var row3Intake: PathChain? = null;

    var toRow1: PathChain? = null;
    var toRow2: PathChain? = null;
    var toRow3: PathChain? = null;
    var toRamp: PathChain? = null;
    var firstShot: PathChain? = null;
    var launch: PathChain? = null;
    var intakeHead = 0.0;

    init {

    }
    fun mirrorPaths(){
        startFromClose = startFromClose.mirror();
        startFromFar = startFromFar.mirror();
        start = start.mirror()
        launchZone = launchZone.mirror()

        row1 = row1.mirror()
        row1End = row1End.mirror()

        row2 = row2.mirror()
        row2End = row2End.mirror()

        row3 = row3.mirror()
        row3End = row3End.mirror()

        ramp = ramp.mirror()
        fromRamp = fromRamp.mirror()
       // intakeHead = 180.0;
    }
    fun buildPaths(){
        row1Intake = robot.follower.pathBuilder()
            .addPath(
            BezierLine(
                row1,
                row1End
            ))
            .setLinearHeadingInterpolation(intakeHead,intakeHead)
            .build()
        row2Intake = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    row2,
                    row2End
                ))
            .setLinearHeadingInterpolation(intakeHead,intakeHead)
            .build()
        row3Intake = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    row3,
                    row3End
                ))
            .setLinearHeadingInterpolation(intakeHead,intakeHead)
            .build()



        if(Constants.zoneType == Constants.ZoneType.Closezone){
            closeZoneAuto()
        }else if(Constants.zoneType == Constants.ZoneType.Farzone){
            farZoneAuto()
        }

        toRow1 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    launchZone,
                    Pose(row1.x-10,row1.y),
                    row1
                ))
            .setLinearHeadingInterpolation(90.0, intakeHead)
            .build()
        toRow2 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    launchZone,
                    Pose(row2.x-10,row2.y),
                    row2
                ))
            .setLinearHeadingInterpolation(intakeHead,intakeHead)
            .build()
        toRow3 = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    launchZone,
                    Pose(row3.x-10,row3.y),
                    row3
                ))
            .setLinearHeadingInterpolation(intakeHead,intakeHead)
            .build()

        toRamp = robot.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    fromRamp,
                    Pose(ramp.x-10,ramp.y),
                    ramp
                ))
            .setLinearHeadingInterpolation(intakeHead,intakeHead)
            .build()
        launch = robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    row1End,
                    launchZone,
                )
            )
            .setLinearHeadingInterpolation(intakeHead, 90.0)
            .build()
    }
    fun farZoneAuto(){
        start = Pose(92.0,7.5,0.0)
        launchZone = Pose(92.0,7.5,0.0);
        fromRamp = row2End;

    }
    fun closeZoneAuto(){
        start = startFromClose;
        launchZone = Pose(96.0,96.0,0.0)
        fromRamp = row3End;
        firstShot = robot.follower.pathBuilder()
            .addPath(
                BezierLine(Pose(122.500, 121.000), Pose(110.000, 121.000))
            )
            .setLinearHeadingInterpolation(0.0, 0.0)
            .build()
    }
    fun toLaunch(follower : Follower) : PathChain{
        return robot.follower.pathBuilder()
            .addPath(
                BezierLine(
                    follower.pose,
                    launchZone,
                )
            )
            .setLinearHeadingInterpolation(0.0, 90.0)
            .build()

    }


}