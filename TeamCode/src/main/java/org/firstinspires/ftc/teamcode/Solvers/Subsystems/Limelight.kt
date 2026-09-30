package org.firstinspires.ftc.teamcode.Solvers.Subsystems

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.Globals.Constants
import org.firstinspires.ftc.teamcode.Globals.Robot
import org.firstinspires.ftc.teamcode.Solvers.Opmodes.Drawing
import org.firstinspires.ftc.teamcode.helpers.controllers.FusionLocalizer
import org.firstinspires.ftc.teamcode.next.filters.kalmanFilter
import kotlin.math.PI

@Config
class Limelight : SubsystemBase() {


    private var lastX = 0.0
    private var lastY = 0.0
    val poseFilter : PoseFilter = PoseFilter();
    val headingFilter: Filter = Filter();

    lateinit var ll: Limelight3A
    @JvmField var truePose: Pose = Pose()
    @JvmField var limelightOn = true

    var botposeHeading = 0.0;
    init {
        ll = robot.ll
        ll.pipelineSwitch(0)
        ll.setPollRateHz(100)
        ll.start()
        follower = robot.follower;

    }
    companion object {

        private val robot: Robot = Robot.getInstance()
        lateinit var follower: Follower;

        @JvmField var autoRunning = false;

        @JvmField var followerCreated = false;
        fun createFollower(hwMap: HardwareMap){
//            fusionLocalizer = FusionLocalizer(
//                PinpointLocalizer(hwMap, Constants.localizerConstants),
//                Pose(500.0, 500.0, 500.0),
//                Pose(2000.0, 200.0, 200.0),
//                Pose(1000.0, 1000.0, 0.1),
//                50
//            )
//            follower = FollowerBuilder(Constants.followerConstants,hwMap)
//                .pathConstraints(Constants.pathConstraints)
//                .setLocalizer(fusionLocalizer)
//                .mecanumDrivetrain(Constants.driveConstants)
//                .build()
//            follower.setStartingPose(robot.follower.pose)
//            followerCreated = true;
//            return follower;
        }
    }
    fun getTx(): Double? {
        val r = grabResultData() ?: return 0.0

        return r.fiducialResults[0].targetXDegrees
    }

    fun grabResultData(): LLResult? {
        val lR = ll.latestResult
        return if (lR != null && lR.isValid) lR else null
    }
    fun megatag2(): Pose? {
        val lR = grabResultData() ?: return null
        if (lR.fiducialResults.isEmpty()) return null

        ll.updateRobotOrientation(Math.toDegrees(follower.heading)+90)

        robot.telemetry.addData("LL Heading", lR.botpose_MT2.orientation.yaw)

        val botpose = lR.botpose_MT2 ?: return null
        botposeHeading = lR.botpose.orientation.yaw
        return  Pose(
            (botpose.position.y * 39.37) + 72,
            -(botpose.position.x* 39.37) + 72,
        )
    }
    fun resetHeading(){
        robot.follower.heading = Math.toRadians(headingFilter.x);
    }
    fun kalman() {
        follower.update()
//
        //kalman implementation:
        truePose = poseFilter.updateFilteredVelocities(follower.pose)
        // if(Constants.ALLIANCE_COLOR == "RED") return
        headingFilter.updateFilteredVelocities(Math.toDegrees(truePose.heading))
        if (!limelightOn) return

        val visionPose = megatag2() ?: return
        val fP = poseFilter.updateFilteredVelocities(visionPose);
        val filteredHeading = headingFilter.updateFilteredVelocities(botposeHeading-90)
        robot.follower.setX(fP.x)
        //robot.follower.heading = Math.toRadians(filteredHeading);
        robot.follower.setY(fP.y);
        truePose = fP;

//
//        follower.pose = Pose(
//            kx.x,
//            ky.x,
//            follower.heading
//        )
    }

    override fun periodic() {
        follower.update()
        //kalman()

    }
    fun getOffset() : Double{
        return grabResultData()?.tx ?: 0.0;
    }
}