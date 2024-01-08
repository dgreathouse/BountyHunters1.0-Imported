//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Tramper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.ICommand;
import frc.robot.subsystems.TramperSubsystem;

public class TramperDefaultCommand extends Command implements ICommand {
  TramperSubsystem m_tramper;
  /** Creates a new ExtensionDefaultCommand. */
  public TramperDefaultCommand(TramperSubsystem _subsystem) {
    m_tramper = _subsystem;
    addRequirements(m_tramper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public void updateDashboard() {

  }
}
