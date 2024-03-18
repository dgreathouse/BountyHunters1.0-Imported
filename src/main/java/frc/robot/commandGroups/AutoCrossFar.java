//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoCrossFar extends SequentialCommandGroup {
  /** Creates a new AutoBlueLeft. */
  public AutoCrossFar(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    // allianceSign is a mutilpier to change the direction of an X, Y Pose or an Angle.
    // By default this play is setup for the blue side and red some values need to be multiplied by -1.0 to change the direction.
    // Pose X positive is away from the drivers and Y positive is to the right.
    // The starting Pose for the robot is (0,0) (X,Y)

    addCommands(
       // Four Note
      
      new AutoDriveOdometry(_drive, new Pose2d(3.5,0,new Rotation2d(Math.toDegrees(0))), 2)       // Cross Line Far
      
      
    );
  }
}
