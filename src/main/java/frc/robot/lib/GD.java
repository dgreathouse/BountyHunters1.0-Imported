//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Global data file is available to all modules. Calibrations go in k.java.
 * This file is used for data that is easier to access than having the instance of the owner.
 * If this file is used the basic concept is that only one and only one module can write to the output.
 * Therefore all other modules can read from here to get the values.
 */
public class GD {
    public static double G_Intake_Speed = 0;
    public static RobotMode G_RobotMode = RobotMode.BOOT;
    public static double G_ShooterAngle = k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG;
    public static double G_ShooterSpeed = 0.0;
    public static boolean G_ShooterIsFlipperRetracted = true;
    public static TargetAngle G_RobotTargetAngle = new TargetAngle(45);
    public static Alliance G_Alliance = Alliance.Blue;
}
