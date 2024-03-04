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
    addCommands( // Yay, explanation!

      new ShooterSetShotCommand(_shooter, 0.65, 0), // Fire up the shooter.
      new AutoDriveTimeVel(_drive,2.15, 0,0, 1.7, 1 , 0.5, false), // Drive forward speedy out of the autonamous start area.
      new AutoDriveTimeVel(_drive,0, 0,-35, 0.7, 0 , 0, false), // Stop AND set the face the robot towards the speaker.
      new InstantCommand(_shooter::setFlipperExtended, _shooter), // FIRE!!!! (Extend the flippers)
      new AutoDriveDelayCommand(_drive, 0.5),
      new AutoDriveTimeVel(_drive,3, -28.5,-28.5, 1.3, 0.5 , 0, false), // Speed away breaking several traffic laws.
      new InstantCommand(_shooter::setFlippersRetracted, _shooter), // Pull back the flipper bois.
      new ShooterSetShotCommand(_shooter, 0, 0),
      new InstantCommand(_intake::spinOn, _intake), // Make intake spinny.
      new AutoDriveTimeVel(_drive,1.5, -28.5,-28.5, 1.5, 0 , 0.75, true), // Go and suck up the note. This looks awsome by the way.
      new ShooterSetShotCommand(_shooter, 0.65, 0),
      new AutoDriveTimeVel(_drive,3, 156,-45, 1.8, 0.5 , 0, false), // Drive away from the note towards firing position.
      new AutoDriveTimeVel(_drive,0, 0,-40, 0.7, 0 , 0, false), // Stop and face speaker.
      new InstantCommand(_intake::spinOff, _intake), // No more intake spinny.
      new InstantCommand(_shooter::setFlipperExtended, _shooter), // FIREEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!! (Extend the flippers again)
      new AutoDriveDelayCommand(_drive, 1),
      new ShooterSetShotCommand(_shooter, 0, 0),
      new InstantCommand(_shooter::setFlipperExtended, _shooter),
      new AutoDriveTimeVel(_drive,1, 20,20, 4, 0.5 , 0, false)

      // TO BE CONTINUED...
    );
  }
}
