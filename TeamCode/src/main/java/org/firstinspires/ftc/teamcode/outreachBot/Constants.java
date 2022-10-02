package org.firstinspires.ftc.teamcode.outreachBot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Drive motor
    public static final double TICKS_PER_REV = 751.8;
    public static final double CIRCUMFERENCE_IN_INCHES = 100 / 25.4 * Math.PI;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / CIRCUMFERENCE_IN_INCHES;

    // Autonomous turn PID
    public static double kR = 0.084; // PID turn kR
    public static double kD = 0.0072; // PID turn kD

    public static double joyStraight = 0.6;
    public static double joyTurn = 0.5;

    public static double liftTop = 0.28;
    public static double liftDrive = 0.94;
    public static double liftBot = 1;
    public static double liftRatio = 0.001;

    public static double clawOpenB = 0.2;
    public static double clawCloseB = 0.5;
    public static double clawOpenA = 0.23;
    public static double clawCloseA = 0.53;

    public static double dpadStraight = 0.15;
    public static double dpadSide = 0.3;
    public static int buttonDelay = 6;

    public static int changeThresh = 128;
    public static int colorThresh = 200;
    public static int tlx = 0; // Top left x for rectangle
    public static int tly = 0; // Top left y for rectangle
    public static int brx = 100; // Bottom right x for rectangle
    public static int bry = 100; // Bottom right y for rectangle

    public static int leftBoundary = 550; // left side of detection zone
    public static int rightBoundary = 750; // right side of detection zone
    public static int middleLine = 618; // detection line y coordinate
}
