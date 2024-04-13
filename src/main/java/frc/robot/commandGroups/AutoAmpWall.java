//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveDelayCommand;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.AutoDriveRotateCommand;
import frc.robot.commands.Shooter.FlipperSetCommand;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.lib.FlipperStates;
import frc.robot.lib.ShooterState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoAmpWall extends SequentialCommandGroup {
  public AutoAmpWall(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    
    addCommands(
      new ShooterSetShotCommand(_shooter, ShooterState.PODIUM),                                                                                         // Set Shooter Speed
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(36),Units.inchesToMeters(6.1),new Rotation2d(Math.toDegrees(0))), 4),               // Drive to side
      new AutoDriveRotateCommand(_drive, 40, 0.5),                                                                                                      // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                                             // Shoot Note #1
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                                           // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                                              // Set flippers back
      new AutoDriveRotateCommand(_drive, 0, .5),                                                                                                        // Rotate to Straight
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(36),Units.inchesToMeters(16),new Rotation2d(Math.toDegrees(0))), 4), 
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(64),Units.inchesToMeters(16),new Rotation2d(Math.toDegrees(0))), 4),
      new InstantCommand(_intake::spinOn), 
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(300),Units.inchesToMeters(3),new Rotation2d(Math.toDegrees(0))), 4),             // Drive back to Note 
      new ShooterSetShotCommand(_shooter, ShooterState.FEED), 
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                                             // Shoot Note #2
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(300),Units.inchesToMeters(-65.00),new Rotation2d(Math.toDegrees(0))), 4),        // Drive at angle to 3rd Note
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                                              // Set flippers back
      new InstantCommand(_intake::spinOff),                                                                                                             // Turn off spinners
      new ShooterSetShotCommand(_shooter, ShooterState.OFF)                                                                                             // Turn off shooter
    );
  }
}
