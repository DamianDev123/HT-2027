package org.firstinspires.ftc.teamcode.Globals;
//import com.acmerobotics.dashboard.config.Config;
public class Constants {
    public static OpModeType OP_MODE_TYPE;
    public enum ZoneType {
        Farzone(1), Closezone(-1);

        private int val;

        ZoneType(int multiplier) {
            val = multiplier;
        }

    }
    public static String ALLIANCE_COLOR = "RED";
    public static ZoneType zoneType = ZoneType.Farzone;
    public static double LAUNCHER_MAX_VELOCITY = 1900; // Ticks/second

    public static boolean autoInitialized = false;
    public static boolean telop = false;
    public static boolean shootingWhileMoving = false;
    public static Double NearDistance = 5.0;
    public enum OpModeType {
        AUTO,
        TELEOP
    }
}
