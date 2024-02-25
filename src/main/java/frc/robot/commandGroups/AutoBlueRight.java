//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.ShooterSetShot;
import frc.robot.lib.Shots;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoBlueRight extends SequentialCommandGroup {
  /** Creates a new AutoBlueRight. */
  ShooterSubsystem m_shooterSubsystem;
  public AutoBlueRight() {

    // addCommands(new AutoDriveTimeVel(), new AutoRotateCommand());
    addCommands(
      new ShooterSetShot(m_shooterSubsystem, Shots.Blue1)
    );
  }
}
