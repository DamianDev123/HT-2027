package org.firstinspires.ftc.teamcode.Globals;

import com.pedropathing.math.Pose;

//import com.acmerobotics.dashboard.config.Config;
public class Constants {
    public static OpModeType OP_MODE_TYPE;
    public static Pose redStartingPose = new Pose(34,9, Math.PI/2);
    public static Pose blueStartingPose = new Pose(111,133,-Math.PI/2);

    public static String ALLIANCE_COLOR = "RED";
    public static double LAUNCHER_MAX_VELOCITY = 2500; // Ticks/second
    public static Pose[] redHive = new Pose[] {new Pose(58.75,59.8),new Pose(58.75,80)};
    public static Pose[] blueHive = new Pose[] {new Pose(85.5,59.8),new Pose(85.5,80)};
    public enum OpModeType {
        AUTO,
        TELEOP
    }
}
