package org.firstinspires.ftc.teamcode.Globals;

import com.pedropathing.math.Pose;

//import com.acmerobotics.dashboard.config.Config;
public class Constants {
    public static OpModeType OP_MODE_TYPE;
    public static Pose redStartingPose = new Pose(30,9, Math.PI/2);
    public static Pose blueStartingPose = new Pose(111,133,-Math.PI/2);

    public static String ALLIANCE_COLOR = "BLUE";
    public static double LAUNCHER_MAX_VELOCITY = 1900; // Ticks/second
    public static Pose[] redHive = new Pose[] {new Pose(58.75,51.8),new Pose(58.75,90)};
    public static Pose[] blueHive = new Pose[] {new Pose(85.5,51.8),new Pose(85.5,90)};
    public enum OpModeType {
        AUTO,
        TELEOP
    }
}
