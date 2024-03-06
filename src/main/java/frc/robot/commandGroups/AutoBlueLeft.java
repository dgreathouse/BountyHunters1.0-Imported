//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveRotateCommand;
import frc.robot.commands.Drive.AutoDriveTimeVel;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoBlueLeft extends SequentialCommandGroup {
  /** Creates a new AutoBlueLeft. */
  public AutoBlueLeft(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {

    // addCommands(new AutoDriveTimeVel(), new AutoRotateCommand());
    addCommands(
      new AutoDriveTimeVel(_drive,1.5, 0,0, 1.75, 0.2 , 0.3, false, false),
      new AutoDriveRotateCommand(_drive, -42, 2),
      new AutoDriveTimeVel(_drive,1.5, -8.5,-8.5, 3.5, 0.2 , 0.3, false, false),
      new AutoDriveTimeVel(_drive,1.5, 171.5,-8.5, 3.5, 0.2 , 0.3, false, false), 
      new AutoDriveRotateCommand(_drive, -42, 2)
      ///new AutoDriveTimeVel(_drive,1.5, 0,0, 3, 0.2 , 0.3, false, false)
    );
  }
}
