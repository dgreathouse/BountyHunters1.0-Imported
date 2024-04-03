//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
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
    double upPerOut = RobotContainer.s_operatorController.getR2Axis() + 1;
    double downPerOut = RobotContainer.s_operatorController.getL2Axis() + 1;
    double perOut = upPerOut - downPerOut;

    if(Math.abs(perOut) > 0.1){
      GD.G_ClimberPerOut = perOut;
    }else {
      GD.G_ClimberPerOut = 0.0;
    }
    SmartDashboard.putNumber("Climber Percent Out", perOut);
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
