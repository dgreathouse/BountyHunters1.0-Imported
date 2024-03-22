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
import frc.robot.lib.ShooterState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoCrossFarGetNote extends SequentialCommandGroup {
  /** Creates a new AutoBlueLeft. */
  public AutoCrossFarGetNote(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    // Both AutoDriveOdometry and AutoDriveRotate will handle the sign for Red vs Blue
    // By default this play is setup for the blue side and red some values need to be multiplied by -1.0 to change the direction.
    // Pose X positive is away from the drivers and Y positive is to the left.
    // The starting Pose for the robot is (0,0) (X,Y)

    addCommands(
       // Four Note
      new ShooterSetShotCommand(_shooter, ShooterState.FEED),                                                     // Set Shooter Speed High
      new FlipperSetCommand(_shooter, FlipperStates.PRELOAD),                                                       // Preload flippers
      new AutoDriveOdometry(_drive, new Pose2d(4.5,-1.63,new Rotation2d(Math.toDegrees(0))), 3.5),      // Go to Mid Area
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                           // Shoot Note
      new InstantCommand(_intake::spinOn),                                                            // Turn on Intake
      new FlipperSetCommand(_shooter, FlipperStates.BACK),                                                          // Set flippers back
      new AutoDriveOdometry(_drive, new Pose2d(6.8,-2.47,new Rotation2d(Math.toDegrees(0))), 3.5),                 // Drive to Note
      new AutoDriveDelayCommand(_drive, 1),                                                                         // Delay for note to come in
      new InstantCommand(_intake::spinOff),                                                                         // Turn off intake
      new AutoDriveOdometry(_drive, new Pose2d(4.5,-1.63,new Rotation2d(Math.toDegrees(0))), 1.65),      // Drive to Mid area
      new ShooterSetShotCommand(_shooter, ShooterState.RIGHT),                                                    // Spin up the shooter
      new AutoDriveOdometry(_drive, new Pose2d(0.96,0,new Rotation2d(Math.toDegrees(0))), 1.65),      // Drive to Shot
      new AutoDriveRotateCommand(_drive, -45, 1),                                               // Rotate to Speaker
      new FlipperSetCommand(_shooter, FlipperStates.SHOOT),                                                         // Shoot Note
      new AutoDriveDelayCommand(_drive, 1),                                                                         // Delay for Note shot
      new ShooterSetShotCommand(_shooter, ShooterState.OFF),                                                         // Set Shooter Speed OFF
      new FlipperSetCommand(_shooter, FlipperStates.BACK)                                                          // Set flippers back
    );
  }
}
