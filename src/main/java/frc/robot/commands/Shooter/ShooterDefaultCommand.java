//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.GD;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.lib.k;

public class ShooterDefaultCommand extends Command{
  ShooterSubsystem m_shooter;
  /** Creates a new ShooterDefaultCommand. */
  public ShooterDefaultCommand(ShooterSubsystem _subsystem) {
    m_shooter = _subsystem;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_shooter.spin(GD.G_ShooterSpeed);
      m_shooter.rotate(GD.G_ShooterAngle);
      if(k.OI.OPERATOR_SHOOT_NOTE.getAsBoolean()) {
        m_shooter.ShootNote(true);
      } else {
        m_shooter.ShootNote(false);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
