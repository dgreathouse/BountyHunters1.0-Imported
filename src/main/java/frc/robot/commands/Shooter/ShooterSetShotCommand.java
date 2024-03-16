//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Shooter;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.GD;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterSetShotCommand extends InstantCommand{
  ShooterSubsystem m_shooter;
  double m_angle;
  double m_speed;


  public ShooterSetShotCommand(ShooterSubsystem _shooter, double _speed, double _angle) {
    m_shooter = _shooter;
    m_angle = _angle;
    m_speed = _speed;
    addRequirements(_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called once since this is a InstantCommand
  @Override
  public void execute() {
    GD.G_ShooterSpeed = m_speed;
  }

}
