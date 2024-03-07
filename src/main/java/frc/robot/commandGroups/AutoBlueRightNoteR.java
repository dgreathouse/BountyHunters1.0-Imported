//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveDelayCommand;
import frc.robot.commands.Drive.AutoDriveRotateCommand;
import frc.robot.commands.Drive.AutoDriveTimeVel;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.lib.Vision.OrangePi5Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoBlueRightNoteR extends SequentialCommandGroup {
  /** Creates a new AutoBlueLeft. */
  public AutoBlueRightNoteR(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {

    // addCommands(new AutoDriveTimeVel(), new AutoRotateCommand());
    addCommands(
      new ShooterSetShotCommand(_shooter, 0.68, 0), // Fire up the shooter.
      new AutoDriveTimeVel(_drive,1.4, 0,0, 1.75, 0.2 , 0.3, false,  false, false),
      
      new AutoDriveRotateCommand(_drive, -44, 2),
      new InstantCommand(_shooter::setFlipperExtended, _shooter),
      new AutoDriveDelayCommand(_drive, 1.5), 
      new AutoDriveRotateCommand(_drive, -17.5, 2),
      new InstantCommand(_intake::spinOn, _intake),
      new InstantCommand(_shooter::setFlippersRetracted, _shooter),
      new AutoDriveTimeVel(_drive,2, -17.5,-17.5, 1, 0.2 , 0.0, false,   false, false),
      new AutoDriveTimeVel(_drive,2, -17.5,-17.5, 2, 0.0 , 0.3, true,   false, false),
      new AutoDriveDelayCommand(_drive, 0.8),
      new AutoDriveTimeVel(_drive,2, 165,-44, 3, 0.2 , 0.3, false,   false, false),
      new AutoDriveTimeVel(_drive,1, 45,-44, 0, 0.1 , 0.1, false,   true, false),
      new InstantCommand(_shooter::setFlipperExtended, _shooter),
      new AutoDriveDelayCommand(_drive, 1.2)
      //new AutoDriveTimeVel(_drive,1.5, -8.5,-8.5, 3.5, 0.2 , 0.3, false, false),
      //new AutoDriveTimeVel(_drive,1.5, 171.5,-8.5, 3.5, 0.2 , 0.3, false, false), 
      //new AutoDriveRotateCommand(_drive, -42, 2)
      ///new AutoDriveTimeVel(_drive,1.5, 0,0, 3, 0.2 , 0.3, false, false)
    );
  }
}
