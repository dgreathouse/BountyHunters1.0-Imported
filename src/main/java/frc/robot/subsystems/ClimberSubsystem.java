//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ClimberSubsystem extends SubsystemBase  implements ISubsystem{
  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  public void updateDashboard() {
    SmartDashboard.putNumber("Climber Left", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Right", rightMotor.getEncoder().getPosition());
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.CLIMBER_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.CLIMBER_RIGHT, MotorType.kBrushless);
    
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    initialize();
  }
  private void initialize(){
    RobotContainer.subsystems.add(this);
  }
  public double getRotations(){
    double leftRotations = leftMotor.getEncoder().getPosition();
    double rightRotations = rightMotor.getEncoder().getPosition();
    double avg = (Math.abs(rightRotations) + Math.abs(leftRotations)) /2.0;
    return avg;
  }

  @Override
  public void periodic() {
    if(GD.G_ClimberVoltageMode){
      if(GD.G_ClimberPerOut <= 0){
        if (leftMotor.getEncoder().getPosition() < k.CLIMBER.LIMIT_UP_ROTATIONS || rightMotor.getEncoder().getPosition() <  k.CLIMBER.LIMIT_UP_ROTATIONS){
          GD.G_ClimberPerOut = 0.0;
        }
      }else if (leftMotor.getEncoder().getPosition() > k.CLIMBER.LIMIT_DOWN_ROTATIONS || rightMotor.getEncoder().getPosition() >  k.CLIMBER.LIMIT_DOWN_ROTATIONS){
        GD.G_ClimberPerOut = 0.0;
      }
      //leftMotor.setVoltage(GD.G_ClimberPerOut * 5);
      //rightMotor.setVoltage(GD.G_ClimberPerOut * 5);
    }else{
      
    }

    // This method will be called once per scheduler run
  }
}
