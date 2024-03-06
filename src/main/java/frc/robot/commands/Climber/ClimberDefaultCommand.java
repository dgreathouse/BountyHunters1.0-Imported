//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.k;
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
    double upVoltage = RobotContainer.s_operatorController.getR2Axis() * k.ROBOT.BATTERY_MAX_VOLTS;
    double downVoltage = RobotContainer.s_operatorController.getL2Axis() * k.ROBOT.BATTERY_MAX_VOLTS;
    double voltage = upVoltage - downVoltage;
    // double position = m_climber.getRotations();

    double setVoltage = 0;
    // if(voltage >= 0 && position > k.CLIMBER.LIMIT_UP_ROTATIONS){
    //   m_climber.setVoltage(0);
    // } else if(voltage < 0 && position < k.CLIMBER.LIMIT_DOWN_ROTATIONS){
    //   m_climber.setVoltage(k.CLIMBER.HOLD_VOLTAGE);
    // }else {
    //   m_climber.setVoltage(voltage);
    // }
    
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
