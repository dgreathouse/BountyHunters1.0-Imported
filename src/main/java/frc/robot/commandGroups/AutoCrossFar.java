//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoCrossFar extends SequentialCommandGroup {
  /** Creates a new AutoBlueLeft. */
  public AutoCrossFar(DrivetrainSubsystem _drive, ShooterSubsystem _shooter, IntakeSubsystem _intake) {

    addCommands(
       // Four Note
      
      new AutoDriveOdometry(_drive, new Pose2d(Units.inchesToMeters(278) ,0,new Rotation2d(Math.toDegrees(0))), 4)       // Cross Line Far
      
      
    );
  }
}
