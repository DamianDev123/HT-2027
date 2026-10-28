package org.firstinspires.ftc.teamcode.Solvers.Subsystems

import com.seattlesolvers.solverslib.command.Subsystem

class Turret : Subsystem {
    /*
 * FTC TURRET AIMING SYSTEM TODO
 *
 * PHASE 1: ENCODER SETUP
 * TODO: Configure turret motor and encoder.
 * TODO: Read current encoder position in ticks.
 * TODO: Calculate encoder ticks per degree using encoder resolution and gear ratio.
 * TODO: Convert encoder ticks to turret angle in degrees.
 * TODO: Establish turret zero position and encoder offset.
 * TODO: Handle angle wrapping and shortest valid rotation.
 * TODO: Test manual rotation and verify encoder readings.
 *
 * PHASE 2: PID POSITION CONTROL
 * TODO: Create a turret subsystem.
 * TODO: Implement setTargetAngle(angle: Double).
 * TODO: Convert target angle to Servo angle,
 * TODO: Test multiple target angles and verify settling accuracy.
 *
 * PHASE 3: ROBOT LOCALIZATION
 * TODO: Connect turret subsystem to Pinpoint or Pedro Pathing.
 * TODO: Read robot field position (x, y).
 * TODO: Read robot heading.
 * TODO: Read field-relative velocity (vx, vy).
 * TODO: Read robot angular velocity if available.
 * TODO: Ensure all measurements use consistent units.
 * TODO: Verify localization and velocity readings while driving.
 *
 * PHASE 4: STATIC GOAL AIMING
 * TODO: Define the goal's field coordinates (goalX, goalY).
 * TODO: Calculate dx = goalX - robotX.
 * TODO: Calculate dy = goalY - robotY.
 * TODO: Calculate absolute goal angle using atan2(dy, dx).
 * TODO: Subtract robot heading to get the turret-relative angle.
 * TODO: Apply turret mounting offset and encoder zero offset.
 * TODO: Normalize the target angle to the turret's valid range.
 * TODO: Pass the calculated angle to setTargetAngle().
 * TODO: Continuously update the target as the robot moves.
 * TODO: Test aiming from different field positions and headings.
 *
 * PHASE 5: MOTION COMPENSATION
 * TODO: Calculate robot velocity relative to the field.
 * TODO: Estimate projectile flight time based on distance and shooter data.
 * TODO: Predict robot position at projectile release or interception.
 * TODO: Calculate a lead angle to compensate for robot movement.
 * TODO: Account for robot angular velocity when calculating turret-relative aim.
 * TODO: Account for shooter velocity and projectile flight physics if needed.
 * TODO: Filter noisy velocity readings without adding excessive delay.
 * TODO: Test while driving forward, reversing, strafing, and rotating.
 * TODO: Tune prediction to reduce moving-shot error.
 *
 * PHASE 6: TELEOP INTEGRATION
 * TODO: Integrate turret subsystem into TeleOp.
 * TODO: Add automatic aiming toggle.
 * TODO: Preserve manual turret control as a fallback.
 * TODO: Update PID and aiming calculations continuously in the OpMode loop.
 * TODO: Handle invalid localization or encoder readings safely.
 * TODO: Add limits to prevent turret cable twisting or mechanical collisions.
 * TODO: Test at multiple distances and robot velocities.
 * TODO: Record aiming error and time required to settle.
 * TODO: Test repeated shots while driving and strafing.
 *
 * RECOMMENDED BUILD ORDER:
 * 1. Encoder readings
 * 2. PID position control
 * 3. Static goal aiming
 * 4. Localization integration
 * 5. Velocity-based motion compensation
 * 6. Moving-shot testing and tuning
 */
    init {
    }
    fun setAngle(){
        // TODO: Write method to set Target Angle
    }
    override fun periodic() {

    }

//    fun alignTurret(xInput: Double, yInput: Double, heading: Double, blue: Boolean): Double {
//        val RobotHeadingDeg = Math.toDegrees(heading)
//        val goalPose = if (blue) robot.shootingWhileMoving.getExpectedPose2(Constants.DblueGoalPose) else robot.shootingWhileMoving.getExpectedPose2(
//            Constants.DredGoalPose)
//
//        val goalX = goalPose.x
//        val goalY = goalPose.y
//
//        val x = turretOffsetX + xInput
//        val y = turretOffsetY + yInput
//
//        val angleToGoal = Math.toDegrees(atan2(goalX - x, goalY - y))
//        var turretAngle = angleToGoal + RobotHeadingDeg - 90
//        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180
//
//        return turretAngle;
//
//    }
//
//    fun normalizeDegrees(angle: Double): Double {
//        var angle = angle % 360;
//        if(angle>180){
//            angle -=360
//        }else if(angle <= -180){
//            angle +=360;
//        }
//        return angle;
//    }
    //
}

