//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveDelayCommand;
import frc.robot.commands.Drive.AutoDriveRotateCommand;
import frc.robot.commands.Drive.AutoDriveTimeVel;
import frc.robot.commands.Drive.AutoDriveTimeVelToAprilArea;
import frc.robot.commands.Drive.AutoDriveTimeVelToAprilYaw;
import frc.robot.commands.Drive.AutoDriveTimeVelToNote;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRedLeftShootPark extends SequentialCommandGroup {
  /** Creates a new AutoRedLeft. */
  public AutoRedLeftShootPark(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    // addCommands(new AutoDriveTimeVel(), new AutoRotateCommand());
    addCommands(
      new ShooterSetShotCommand(_shooter, 0.7, 0), // Fire up the shooter.
      new AutoDriveTimeVel(_drive,2, 0,0, 2.1, 0.2 , 0.3, false),
      
      new AutoDriveRotateCommand(_drive, 48, 1),
      new InstantCommand(_shooter::setFlipperExtended, _shooter),
      new AutoDriveDelayCommand(_drive, 1), 
      new AutoDriveRotateCommand(_drive, 17.5, 1.0),
      new InstantCommand(_intake::spinOn, _intake),
      new InstantCommand(_shooter::setFlippersRetracted, _shooter),
      new AutoDriveDelayCommand(_drive, 1.2),
      new AutoDriveTimeVel(_drive, 1, 90, 0, 3, 0.2, 0.2, false),
      new InstantCommand(_shooter::setFlippersRetracted, _shooter),
      new ShooterSetShotCommand(_shooter, 0, 0),
      new InstantCommand(_intake::spinOff, _intake),
      new AutoDriveDelayCommand(_drive, 2)
    );
  }
}
