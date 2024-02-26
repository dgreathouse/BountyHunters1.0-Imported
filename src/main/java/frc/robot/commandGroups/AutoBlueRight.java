//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveTimeVel;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoBlueRight extends SequentialCommandGroup {
  /** Creates a new AutoBlueRight. */
  ShooterSubsystem m_shooterSubsystem;
  public AutoBlueRight(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {

    // addCommands(new AutoDriveTimeVel(), new AutoRotateCommand());
    addCommands(
      new AutoDriveTimeVel(_drive, 0, 0, 0, 0, 0, 0),
      new ShooterSetShotCommand(_shooter, 0, 0),
      new InstantCommand(_intake::spinOn, _intake),
      new InstantCommand(_intake::spinOff, _intake),
      new InstantCommand(_shooter::extendFlippers, _shooter),
      new InstantCommand(_shooter::retractFlippers, _shooter)
    );
  }
}
