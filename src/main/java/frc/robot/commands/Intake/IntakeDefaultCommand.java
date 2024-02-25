//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.lib.k;

public class IntakeDefaultCommand extends Command{
  IntakeSubsystem m_intake;
  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(IntakeSubsystem _subsystem) {
    m_intake = _subsystem;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(k.OI.OPERATOR_INTAKE_SPIN.getAsBoolean()){
      m_intake.spin(1);

    }else {
      m_intake.spin(0);
    }
    //m_intake.rotate(0);
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
