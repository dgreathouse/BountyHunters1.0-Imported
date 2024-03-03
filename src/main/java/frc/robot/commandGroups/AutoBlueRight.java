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
import frc.robot.lib.Vision.OrangePi5Vision;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoBlueRight extends SequentialCommandGroup {
  /** Creates a new AutoBlueRight. */
  ShooterSubsystem m_shooterSubsystem;
  public AutoBlueRight(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake, OrangePi5Vision _vision) {
    
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
    addCommands(

      new AutoDriveTimeVel(_drive,2, 0,0, 1.5, 0.5 , 0.5, false),
      new AutoDriveTimeVel(_drive,0, 0,0, 2, 0 , 0, false),
      new AutoDriveTimeVel(_drive,3, -28,-28, 1.3, 0.5 , 0, false),
      new InstantCommand(_intake::spinOn, _intake),
      new AutoDriveTimeVel(_drive,1.5, -22,-22, 1, 0 , 0.5, true),
      new AutoDriveTimeVel(_drive,3, 175,-45, 1.5, 0.5 , 0, false),
      new AutoDriveTimeVel(_drive,0, 0,0, 2, 0 , 0, false),
      new InstantCommand(_intake::spinOff, _intake)
      
     // new ShooterSetShotCommand(_shooter, 0.15, 40),
      // new AutoDriveRotateCommand(_drive, -40, 2),
     //  new InstantCommand(_shooter::setFlippersRetracted, _shooter)
       //new AutoDriveDelayCommand(_drive, 10)
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new InstantCommand(_shooter::retractFlippers, _shooter),
      // new InstantCommand(_intake::spinOn, _intake),
      
      // new AutoDriveDelayCommand(_drive, 1),
      // new AutoDriveTimeVel(_drive, 3, 180-22,-35, 1.8, 0.5, 0.5)
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new AutoDriveDelayCommand(_drive, 1),
      // new InstantCommand(_intake::spinOff, _intake),
      // new AutoDriveTimeVel(_drive, 0, 0, 0, 0, 0, 0),
      // new InstantCommand(_shooter::retractFlippers, _shooter),
    //   new AutoDriveDelayCommand(_drive, 20)
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new InstantCommand(_shooter::retractFlippers, _shooter),
      // new AutoDriveTimeVel(_drive, 0, 0, 0, 0, 0, 0),

    );
  }
}
