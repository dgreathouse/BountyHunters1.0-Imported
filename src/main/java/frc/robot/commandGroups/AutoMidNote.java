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


public class AutoMidNote extends SequentialCommandGroup {
  /** Creates a new AutoBlueLeft. */
  public AutoMidNote(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {

    
    addCommands(
      new ShooterSetShotCommand(_shooter, ShooterState.PODIUM),                                                                               // Set Shooter Speed High
      new FlipperSetCommand(_shooter, FlipperStates.PRELOAD),                                                                                 // Preload flippers
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(15),Units.inchesToMeters(0),new Rotation2d(Math.toDegrees(0))), 3),       // Drive to shot
      new AutoDriveRotateCommand(_drive, -60, 1),                                                                                           // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                                   // Shoot Note
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                                 // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                                    // Set flippers back
      new ShooterSetShotCommand(_shooter, ShooterState.OFF),                                                                                  // Set Shooter Speed OFF
      new AutoDriveRotateCommand(_drive, 0, 0.5),                                                                                             // Rotate to Straight
      new InstantCommand(_intake::spinOn),                                                                                                          // Turn on the intake
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(200),Units.inchesToMeters(-50),new Rotation2d(Math.toDegrees(0))), 3.5),  // Drive to Note
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(278),Units.inchesToMeters(13),new Rotation2d(Math.toDegrees(0))), 3.5),  // Drive to Note
      new AutoDriveDelayCommand(_drive, 1.5),                                                                                                 // Delay for Note to be grabbed
      new InstantCommand(_intake::spinOff),                                                                                                   // Turn off intake
      // new ShooterSetShotCommand(_shooter, ShooterState.PODIUM),                                                                               // Set Shooter Speed HIGH
      // new FlipperSetCommand(_shooter, FlipperStates.PRELOAD),                                                                                 // Preload flippers
      // new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(200),Units.inchesToMeters(-50) ,new Rotation2d(Math.toDegrees(0))), 3.5),    // Drive to Shot  Red
      // new InstantCommand(_intake::spinOff),
      // new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(38),Units.inchesToMeters(5),new Rotation2d(Math.toDegrees(0))), 3.5),       // Drive to shot
      // new AutoDriveRotateCommand(_drive, -60, 1),                                                                                           // Rotate to Speaker
      // new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                                   // Shoot Note
      // new AutoDriveDelayCommand(_drive, 1),                                                                                                 // Delay for Note shot
      new ShooterSetShotCommand(_shooter, ShooterState.OFF),                                                                                  // Set Shooter Speed OFF
      new FlipperSetCommand(_shooter, FlipperStates.BACK)                                                                                     // Set flippers back
      
    );
  }
}
