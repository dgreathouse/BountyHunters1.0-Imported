//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveDelayCommand;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.AutoDriveRotateCommand;
import frc.robot.commands.Shooter.FlipperSetCommand;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.lib.FlipperStates;
import frc.robot.lib.GD;
import frc.robot.lib.ShooterState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Auto4Note extends SequentialCommandGroup {
  public Auto4Note(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    // GD.G_AllianceSign is a mutilpier to change the direction of an X, Y Pose or an Angle.
    // By default this play is setup for the Blue side and when Red some values need to be multiplied by -1.0 to change the direction or location.
    // Pose X positive is away from the drivers and Y positive is to the left.
    // The starting Pose for the robot is (0,0) (X,Y)

    addCommands(
       // Four Note
      new ShooterSetShotCommand(_shooter, ShooterState.PODIUM),                                                     // Set Shooter Speed
      new AutoDriveOdometry(_drive, new Pose2d(0.8128,0.1524,new Rotation2d(Math.toDegrees(0))), 1),                          // Drive back
      new AutoDriveRotateCommand(_drive, 45 * GD.G_AllianceSign, 1.0),                                               // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                         // Shoot Note #1
      new AutoDriveDelayCommand(_drive, 0.8),                                                                       // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                              // Set flippers back
      new AutoDriveRotateCommand(_drive, 0, 1.0),                                                                   // Rotate to Straight
      new InstantCommand(_intake::spinOn),                                                                          // Turn on the intake
      new AutoDriveOdometry(_drive, new Pose2d(0.8128,-0.357 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 1),      // Drive back to Note at angle
      new AutoDriveOdometry(_drive, new Pose2d(1.6,-0.357 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 1),      // Drive back to Note at angle
      new AutoDriveRotateCommand(_drive, 30 * GD.G_AllianceSign, 1.0),                                               // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                         // Shoot Note #2
      new AutoDriveDelayCommand(_drive, 0.8),                                                                       // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                          // Set flippers back
      new AutoDriveRotateCommand(_drive, 0, 1.0),                                                                   // Rotate to Straight
      new AutoDriveOdometry(_drive, new Pose2d(0.7112,-0.357 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 1),
      new AutoDriveOdometry(_drive, new Pose2d(0.7112,-1.85 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 1),      // Drive forward at angle
      new AutoDriveOdometry(_drive, new Pose2d(1.7,-1.85 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 1),      // Drive back to note at angle
      // new AutoDriveDelayCommand(_drive, 0.8),                                                                       // Delay for Note to be grabbed
      // new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                         // Shoot Note #3
      // new AutoDriveDelayCommand(_drive, 0.8),                                                                       // Delay for Note to be released
      // new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                          // Set flippers back
      // new AutoDriveOdometry(_drive, new Pose2d(0,0 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 2),      // Drive forward at angle
      // new AutoDriveOdometry(_drive, new Pose2d(0,0 * GD.G_AllianceSign,new Rotation2d(Math.toDegrees(0))), 2),      // Drive back to note at angle and robot angle
      // new AutoDriveDelayCommand(_drive, 0.8),                                                                       // Delay for Note to be grabbed
      // new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                         // Shoot Note #4
      // new AutoDriveDelayCommand(_drive, 0.8),                                                                       // Delay for Note to be released
      // new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                          // Set flippers back
      // new InstantCommand(_intake::spinOff)                                                                          // Turn off the intake
      new AutoDriveDelayCommand(_drive, 10)
    );
  }
}
