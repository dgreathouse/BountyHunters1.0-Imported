// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.FlipperStates;
import frc.robot.lib.GD;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlipperSetCommand extends InstantCommand {
  FlipperStates m_state = FlipperStates.BACK;
  public FlipperSetCommand(ShooterSubsystem _shooter, FlipperStates _state) {
    addRequirements(_shooter);
    m_state = _state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GD.G_FlipperState = m_state;
  }
}
