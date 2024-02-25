//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.Drive.AutoDrivetrainDefaultCommand;
import frc.robot.lib.Shots;

public class ShooterSetShot extends Command{
  ShooterSubsystem m_shooter;
  AutoDrivetrainDefaultCommand m_autoDrivetrainDefaultCommand;
  Shots m_shots;

  /**
   * * AutoDriveTimeVel
   * <p>
   * Drive at a set velocity, robot angle and time. A velocity ramp time of 1
   * second will be used if rampEnable is true.
   * 
   * @param _drive       An instance of the drive subsystem
   * @param _velocity    The velocity you want the robot to drive at in meters/sec
   * @param _driveAngle  The Drive angle the chassis should drive at in degrees
   * @param _robotAngle  The Angle the robot should face while driving in degrees
   * @param _timeOut_sec The time to stop driving in seconds.
   * @param _rampEnable  Enable the ramp of velocity at the start.
   */
  public ShooterSetShot(ShooterSubsystem _shooter, Shots _shots) {
    m_shooter = _shooter;
    m_shots = _shots;
    addRequirements(_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_autoDrivetrainDefaultCommand.setShotData(m_shots);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
