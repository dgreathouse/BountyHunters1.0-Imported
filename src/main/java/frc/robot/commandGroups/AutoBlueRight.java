//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveDelayCommand;
import frc.robot.commands.Drive.AutoDriveRotateCommand;
import frc.robot.commands.Drive.AutoDriveTimeVel;
import frc.robot.commands.Shooter.ShooterDefaultCommand;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoBlueRight extends SequentialCommandGroup {
  /** Creates a new AutoBlueRight. */
  ShooterSubsystem m_shooterSubsystem;
  public AutoBlueRight(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    
    /** Auto Blue Right
     *  + Line up robot on right side pointing straight down field
     *  - Set Shooter Spin and Angle
     *  - Rotate Robot to speaker
     *  - Extend flippers
     *  - Delay to let Note shoot
     *  - Set Shooter Spin and Angle to initial position
     *  - Retract flippers
     *  - Turn on Intake
     *  - Drive to center field and get Note next to wall
     *  - Turn off Intake
     *  - Set Shooter Spin and Angle for next shot
     *  - Drive back to Speaker area for shot 
     *  - Extend flippers
     *  - Delay to let Note shoot
     *  - Set Shooter Spin and Angle to initial position
     *  - Retract flippers
     *  - Drive to center field
     */
    super(
      new ParallelCommandGroup(
        new ShooterSetShotCommand(_shooter, .3, 40),
      new ShooterDefaultCommand(_shooter)
      ),
  
      // new AutoDriveRotateCommand(_drive, 0, 0),
      // new InstantCommand(_shooter::extendFlippers, _shooter),
       new AutoDriveDelayCommand(_drive, 10)
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new InstantCommand(_shooter::retractFlippers, _shooter),
      // new InstantCommand(_intake::spinOn, _intake),
       //new AutoDriveTimeVel(_drive, 0, 0, 0, 0, 0, 0)
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new AutoDriveDelayCommand(_drive, 1),
      // new InstantCommand(_intake::spinOff, _intake),
      // new AutoDriveTimeVel(_drive, 0, 0, 0, 0, 0, 0),
      // new InstantCommand(_shooter::retractFlippers, _shooter),
      // new AutoDriveDelayCommand(_drive, 1),
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new InstantCommand(_shooter::retractFlippers, _shooter),
      // new AutoDriveTimeVel(_drive, 0, 0, 0, 0, 0, 0),

    );
  }
}
