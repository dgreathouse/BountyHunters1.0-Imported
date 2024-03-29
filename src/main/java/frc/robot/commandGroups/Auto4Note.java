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
import frc.robot.lib.k;
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
      new InstantCommand(_drive::setAllianceSign, _drive),
      new ShooterSetShotCommand(_shooter, ShooterState.PODIUM),                                                               // Set Shooter Speed
      new AutoDriveOdometry(_drive, new Pose2d(32 * k.CV.INCHES_TO_METERS,6.1 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),                       // Drive to side
      new AutoDriveRotateCommand(_drive, 45, 0.5),                                                        // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                   // Shoot Note #1
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                 // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                    // Set flippers back
      new AutoDriveRotateCommand(_drive, 0, .5),                                                                             // Rotate to Straight
      new InstantCommand(_intake::spinOn),                                                                                    // Turn on the intake
      new AutoDriveOdometry(_drive, new Pose2d(32 * k.CV.INCHES_TO_METERS,-14.05 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),   // Strafe to second note
      new AutoDriveOdometry(_drive, new Pose2d(60 * k.CV.INCHES_TO_METERS,-12.4 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),     // Drive back to Note 
      new AutoDriveRotateCommand(_drive, 30, 0.5),                                                        // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                   // Shoot Note #2
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                 // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                    // Set flippers back
      new AutoDriveRotateCommand(_drive, 0, 0.5),                                                                             // Rotate to Straight
      //new AutoDriveOdometry(_drive, new Pose2d(0.8112,-0.357,new Rotation2d(Math.toDegrees(0))), 4),   // Drive forward
      new AutoDriveOdometry(_drive, new Pose2d(31.94 * k.CV.INCHES_TO_METERS,-64.00 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),     // Strafe to 3rd Note
      new AutoDriveOdometry(_drive, new Pose2d(78.74 * k.CV.INCHES_TO_METERS,-64.00 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),        // Drive back to note
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                 // Delay for Note to be grabbed
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                   // Shoot Note #3
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                 // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                    // Set flippers back
      new AutoDriveOdometry(_drive, new Pose2d(31.93 * k.CV.INCHES_TO_METERS,-122.05 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),     // Drive at angle to be in front to 4th Note
      //
      new AutoDriveOdometry(_drive, new Pose2d(68 * k.CV.INCHES_TO_METERS,-122.05 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),      // Drive forward to get note
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                 // Delay for note to be taken in
      new AutoDriveOdometry(_drive, new Pose2d(68 * k.CV.INCHES_TO_METERS,-68.9 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),      // Drive to cneter get note
      new AutoDriveOdometry(_drive, new Pose2d(75.48 * k.CV.INCHES_TO_METERS,-68.9 * k.CV.INCHES_TO_METERS,new Rotation2d(Math.toDegrees(0))), 4),        // Drive back to note
      //new AutoDriveRotateCommand(_drive, -35, 1),                                                       // Rotate to speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                                   // Shoot Note #4
      new AutoDriveDelayCommand(_drive, 0.5),                                                                                 // Delay for Note to be released
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                                    // Set flippers back
      new InstantCommand(_intake::spinOff),                                                                                   // Turn off spinners
      new ShooterSetShotCommand(_shooter, ShooterState.OFF)                                                                   // Turn off shooter

    );
  }
}
