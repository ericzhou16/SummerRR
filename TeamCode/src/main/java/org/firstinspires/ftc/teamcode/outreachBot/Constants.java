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
}
