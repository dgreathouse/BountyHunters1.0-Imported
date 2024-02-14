//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
import frc.robot.lib.k;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainDefaultCommand extends Command{

  private DrivetrainSubsystem m_drive;
  ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _drivetrain) {
    m_drive = _drivetrain;
    this.setName("DriveDefaultCommand");
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Set local variables to game thumbstick axis values
    double leftY = -RobotContainer.s_driverController.getLeftY();
    double leftX = -RobotContainer.s_driverController.getLeftX();
    double rightX = -RobotContainer.s_driverController.getRightX();
    double rightY = -RobotContainer.s_driverController.getRightY();

    // Limit the inputs for a deadband related to the joystick
    leftY = MathUtil.applyDeadband(leftY, 0.08, 1.0);
    leftX = MathUtil.applyDeadband(leftX, 0.08, 1.0);
    rightY = MathUtil.applyDeadband(rightY, 0.08, 1.0);
    rightX = MathUtil.applyDeadband(rightX, 0.08, 1.0);

    // Set the class variable ChassisSpeeds to the local variables in their
    // appropriate units.
    m_speeds.vxMetersPerSecond = leftY * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.vyMetersPerSecond = leftX * k.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.omegaRadiansPerSecond = rightX * k.DRIVE.MAX_ANGULAR_VELOCITY_RadianPerSec;

    // If correct button is pressed, set the robot angle, shooter angle and shooter speed.
    setShotData(rightX,rightY);
    
    // Call the appropriate drive mode. Selected by the driver controller Square button.
    
    switch (m_drive.getDriveMode()) {
      case FIELD_CENTRIC:
        m_drive.driveFieldCentric(m_speeds);
        break;
      case ROBOT_CENTRIC:
        m_drive.driveRobotCentric(m_speeds);
        break;
      case ANGLE_FIELD_CENTRIC:
        m_drive.driveAngleFieldCentric(m_speeds.vxMetersPerSecond, m_speeds.vyMetersPerSecond);
        break;
      default:
        break;
    }
  }

  /** If right stick X,Y Hyp is >  0.8 then set the target angle
   *  Reset the Shooter angle and speed to default if button not pressed.
   * 
   * Set the following data if the appropriate buttons are pressed
   * Robot Angle to turn to
   * Shooter angle to move to
   * Shooter speed to run at.
   */
  private void setShotData(double _x, double _y) {
    // set the target angle only if the x,y hyp are > deadband
    GD.G_RobotTargetAngle.setTargetAngle(_x, _y);
    // if x,y hyp > deadband reset the shooter angle and speed 
    if(GD.G_RobotTargetAngle.getHyp() > k.DRIVE.TARGET_ANGLE_DEADBAND){
      GD.G_ShooterAngle = 15.0;
      GD.G_ShooterSpeed = 0.0;
    }
    // Handle target angle buttons if pressed to set angles and speeds
    if (RobotContainer.s_driverController.R1().getAsBoolean()) {
      if (RobotContainer.s_driverController.square().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      } else if (RobotContainer.s_driverController.circle().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      } else if (RobotContainer.s_driverController.triangle().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      } else if (RobotContainer.s_driverController.cross().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      }
    } else {
      if (RobotContainer.s_driverController.square().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      } else if (RobotContainer.s_driverController.circle().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      } else if (RobotContainer.s_driverController.triangle().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      } else if (RobotContainer.s_driverController.cross().getAsBoolean()) {
        GD.G_RobotTargetAngle.setTargetAngle(50);
        GD.G_ShooterAngle = 40;
        GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
      }
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public void updateDashboard() {
    SmartDashboard.putNumber("ChassisSpeeds(Command) MPS(X)", m_speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds(Command) MPS(Y)", m_speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds(Command) Deg/Sec", m_speeds.omegaRadiansPerSecond * k.CONVERT.RADIANS_TO_DEGREES);
  }
}
