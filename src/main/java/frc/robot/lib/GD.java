//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib;

/** Global data file is available to all modules. Calibrations go in k.java.
 * This file is used for data that is easier to access than having the instance of the owner.
 * If this file is used the basic concept is that only one and only one module can write to the output.
 * Therefore all other modules can read from here to get the values.
 */
public class GD {
    public static RobotMode G_RobotMode = RobotMode.BOOT;
    public static double G_ShooterAngle = 0.0;
    public static double G_ShooterSpeed = 0.0;
    public static TargetAngle G_RobotTargetAngle = new TargetAngle(45);
}
