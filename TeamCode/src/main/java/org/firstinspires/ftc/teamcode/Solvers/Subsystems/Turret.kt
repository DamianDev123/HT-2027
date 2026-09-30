package org.firstinspires.ftc.teamcode.Solvers.Subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.control.PIDFController
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.ElapsedTime
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import org.firstinspires.ftc.teamcode.Globals.Constants
import org.firstinspires.ftc.teamcode.Globals.Robot
import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Subsystem
import org.firstinspires.ftc.teamcode.Solvers.Opmodes.testOp
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

@Config
class Turret : Subsystem() {
    private val robot: Robot = Robot.getInstance()
    @JvmField
    var inAuto = true;

    var runningAuto = false;
    var override = false;
    @JvmField
    var overrided = 0.0;
    @JvmField
    var forcedPos = Pose(0.0,0.0)

    val velocity = (60/0.110)/2
    var currentPosition = 0.0;
    val posTimer = ElapsedTime();

    companion object {
        @JvmField var tuningOffset = false

        @JvmField var expo = 2;


        @JvmField var kv =0.002;
        @JvmField var backWards = true;
        @JvmField var offsetR = 0.0;
        @JvmField var offsetRBack = -0.0;
        @JvmField var headingOffset = 0.0;
        @JvmField var llCoefficients = PIDFCoefficients(0.0,0.0,0.0,0.0);
        @JvmField var xOffset = 0.0
        @JvmField var yOffset = 0.0
    }
    var follower: Follower = robot.follower
    var turretServo2: ServoEx = robot.turretServo2
    var turretServo1: ServoEx = robot.turretServo1
    private val ppr = 4000
    private val gearRatio = 90.0 / 35.0
    private val ticksPerDeg = (ppr * gearRatio) / 360
    var elapsedTime: ElapsedTime = ElapsedTime();
    @JvmField
    var tolerable = false;
    @JvmField
    var shouldAim = false;
    @JvmField
    var toggle = true;
    var controller = PIDFController(llCoefficients);
    init {
        elapsedTime.startTime()
        elapsedTime.reset()
        posTimer.startTime();
        posTimer.reset();
    }
    fun headingScalar(heading: Double) : Double{
        val x = ((heading + 180) % 360 + 360) % 360 - 180;
        robot.telemetry.addData("heading", heading)
        val y = -0.0000667247*x*x-0.078323*x-0.984867
        return heading+y;
    }
    fun xScalar(x: Double) : Double{

        return x+xOffset;
    }
    var degree = 0.0;
    fun yScalar(y: Double) : Double{
        robot.telemetry.addData("y", y)
        return y-yOffset;
        ;
    }
    fun update() {


        robot.profiler.start("Turret Loop")
        var target = 90.0;
        target = normalizeDegrees(-CalculateGoal());
        if(tuningOffset)
            target = 90.0;
            // 1. Find the "Range" (how many servo units represent 90 degrees)s
        target = Math.toDegrees(MathFunctions.normalizeAngle(Math.toRadians(target)));
        val o = 0.1;
        val x = robot.launcher.distance;
        val of = offsetR+0.000171925*x*x+0.15768*x-12.02042
        //val of = (robot.limelight.getOffset()+currentPosition)-degree;

        degree = -(((target)))+180-of;

        FtcDashboard.getInstance().telemetry.addData("FakeAngle:", degree)
        FtcDashboard.getInstance().telemetry.addData("RealAngle", getPosition(degree))

        val servoPosition = testOp.toServo(degree, backWards)

        turretServo1.set(servoPosition)
        turretServo2.set(servoPosition)

        val error = Math.abs(robot.turretEncoder.position / ticksPerDeg + target)
        val ntolerable = error > 4
        if (ntolerable)
            elapsedTime.reset()
        tolerable = elapsedTime.milliseconds() > 300;
        robot.telemetry.update()

        robot.profiler.end("Turret Loop")

        robot.telemetry.addData("heading:", robot.follower.heading)
    }
    override fun periodic() {
        update()
    }
    fun getPosition(deg: Double) : Double{
        val maxMovement = elapsedTime.milliseconds()*(velocity/1000)
        val error = currentPosition-deg;
        if(Math.abs(error) <= maxMovement){
            currentPosition = deg;
        }else {
            currentPosition += sign(error) *maxMovement;
        }
        elapsedTime.reset()
        return currentPosition;
    }
    fun overrideTurret(p: Pose){
        forcedPos = p;
    }
    fun CalculateGoal(): Double {
        var currentP = robot.CurrentPose;
        val mu = atan2((robot.GoalPose.y - currentP.y), (robot.GoalPose.x - currentP.x))
        val deltaHeading = mu-robot.pose.heading
        return Math.toDegrees(deltaHeading)+180
    }
    fun getPositionAbs(): Double {
        return testOp.toDegree(turretServo1.rawPosition,backWards);
    }
    fun normalizeDegrees(angle: Double): Double {
        var angle = angle % 360;
        if(angle>180){
            angle -=360
        }else if(angle <= -180){
            angle +=360;
        }
        return  angle;
    }
}
