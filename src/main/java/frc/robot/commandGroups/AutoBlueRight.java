//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveDelayCommand;
import frc.robot.commands.Drive.AutoDriveTimeVel;
import frc.robot.commands.Shooter.ShooterSetShotCommand;
import frc.robot.lib.Vision.OrangePi5Vision;
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
    addCommands( // Yay, explanation!

      // new ShooterSetShotCommand(_shooter, 0.65, 0), // Fire up the shooter.
      new AutoDriveTimeVel(_drive,1.5, 0,0, 2, 0.2 , 0.3, false, false) // Drive forward speedy out of the autonamous start area.
      // new AutoDriveTimeVel(_drive,0, 0,-33, 1, 0 , 0, false, true), // Stop AND set the face the robot towards the speaker.
      // // new InstantCommand(_shooter::setFlipperExtended, _shooter), // FIRE!!!! (Extend the flippers)
      // // new AutoDriveDelayCommand(_drive, 0.8),
      // new AutoDriveTimeVel(_drive,3, -24.8,-24.8, 1.3, 0.5 , 0, false, true), // Speed away breaking several traffic laws.
      // // new InstantCommand(_shooter::setFlippersRetracted, _shooter), // Pull back the flipper bois.
      // // new ShooterSetShotCommand(_shooter, 0, 0),
      // // new InstantCommand(_intake::spinOn, _intake), // Make intake spinny.
      // new AutoDriveTimeVel(_drive,1.5, -24.8,-24.8, 1.5, 0 , 0.75, false, false), // Go and suck up the note. This looks awsome by the way.
      // // new ShooterSetShotCommand(_shooter, 0.65, 0),
      // new AutoDriveTimeVel(_drive,3, 155,-24.8, 1.8, 0.5 , 0, false, true), // Drive away from the note towards firing position.
      // new AutoDriveTimeVel(_drive,0, 0,-36, 0.7, 0 , 0, false, true) // Stop and face speaker.
      // // new InstantCommand(_intake::spinOff, _intake), // No more intake spinny.
      // new InstantCommand(_shooter::setFlipperExtended, _shooter), // FIREEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!! (Extend the flippers again)
      // new AutoDriveDelayCommand(_drive, 1),
      // new ShooterSetShotCommand(_shooter, 0, 0),
      // new InstantCommand(_shooter::setFlipperExtended, _shooter),
      // new AutoDriveTimeVel(_drive,0.5, 30,30, 5, 0.5 , 0, false, true),
      // new AutoDriveTimeVel(_drive,0.5, 0,0, 4, 0.5 , 0, false, true)

      // TO BE CONTINUED...
    );
  }
}
