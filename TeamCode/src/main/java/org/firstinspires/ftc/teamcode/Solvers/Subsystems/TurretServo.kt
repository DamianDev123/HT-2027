package org.firstinspires.ftc.teamcode.Solvers.Subsystems

import com.pedropathing.follower.Follower
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import org.firstinspires.ftc.teamcode.Globals.Robot

class TurretServo : SubsystemBase() {
    private val robot: Robot = Robot.getInstance()
    var turretEncoder: Motor.Encoder = robot.turretEncoder
    var turretServo2: ServoEx = robot.turretServo2
    var turretServo1: ServoEx = robot.turretServo1
    var follower: Follower = robot.follower
    companion object Constants {
        @JvmField
        val velPIDCoefficients = PIDCoefficients(0.0,0.0,0.0)
        @JvmField
        val velFFCoefficients = BasicFeedforwardParameters(0.0,0.0,0.0)
        @JvmField
        val posPIDCoefficients = PIDCoefficients(0.0,0.0,0.0)

        @JvmField var targetVel = 0.0;
        @JvmField var targetPos = 0.0;
        @JvmField var tuningVel = false
    }


    val velControl = controlSystem {
        velPid(velPIDCoefficients)
        basicFF(velFFCoefficients)
    }
    val posControl = controlSystem {
        posPid(posPIDCoefficients)
    }


    override fun periodic() {

        update()
    }
    fun update(){
        posControl.goal = KineticState(targetPos);
        var calc = posControl.calculate(KineticState(turretEncoder.position*1.0))
        if(tuningVel)
            calc = targetVel
        velControl.goal = KineticState(0.0,calc)
        setPower(velControl.calculate(KineticState(turretEncoder.position*1.0,turretEncoder.correctedVelocity)))
    }
    fun setPower(power: Double){
        turretServo1.set((power/2)+0.5)
        turretServo2.set((power/2)+0.5)

    }
}