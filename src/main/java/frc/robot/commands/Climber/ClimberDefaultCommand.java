//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDefaultCommand extends Command{
  ClimberSubsystem m_climber;

  /** Creates a new ClimberDefaultCommand. */
  public ClimberDefaultCommand(ClimberSubsystem _subsystem) {
    m_climber = _subsystem;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upVoltage = RobotContainer.s_operatorController.getR2Axis() + 1;
    double downVoltage = RobotContainer.s_operatorController.getL2Axis() + 1;
    double voltage = upVoltage - downVoltage;
    // if(Math.abs(voltage) > 0.5){ // Prevent the climber from creeping up or down.
    //   m_climber.setVoltage(voltage);
    // }
    SmartDashboard.putNumber("Climber Volts", voltage);
    m_climber.setVoltage(voltage);
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
