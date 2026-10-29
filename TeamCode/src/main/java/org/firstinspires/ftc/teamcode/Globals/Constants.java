package org.firstinspires.ftc.teamcode.Globals;

import com.pedropathing.math.Pose;

//import com.acmerobotics.dashboard.config.Config;
public class Constants {
    public static OpModeType OP_MODE_TYPE;
    public static Pose redStartingPose = new Pose(30,9);
    public static Pose blueStartingPose = new Pose(111,133);

    public static String ALLIANCE_COLOR = "RED";
    public static double LAUNCHER_MAX_VELOCITY = 1900; // Ticks/second
    public static Pose[] blueHive = new Pose[] {new Pose(84,51.8),new Pose(84,90)};
    public static Pose[] redHive = new Pose[] {new Pose(84,51.8),new Pose(84,90)};
    public enum OpModeType {
        AUTO,
        TELEOP
    }
}
