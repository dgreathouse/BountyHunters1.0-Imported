//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Shooter;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.GD;
import frc.robot.lib.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterSetShotCommand extends InstantCommand{
  ShooterSubsystem m_shooter;
  ShooterState m_state;


  public ShooterSetShotCommand(ShooterSubsystem _shooter, ShooterState _state) {
    m_shooter = _shooter;
    m_state = _state;
    addRequirements(_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called once since this is a InstantCommand
  @Override
  public void execute() {
    GD.G_ShooterState = m_state;
  }

}
