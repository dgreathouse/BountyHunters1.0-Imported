//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ClimberSubsystem extends SubsystemBase  implements ISubsystem{
  // CANSparkMax leftMotor;
  // CANSparkMax rightMotor;

  public void updateDashboard() {

  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // leftMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.CLIMBER_LEFT, MotorType.kBrushless);
    // rightMotor = new CANSparkMax(k.ROBORIO_CAN_IDS.CLIMBER_RIGHT, MotorType.kBrushless);
    // leftMotor.setIdleMode(IdleMode.kBrake);
    // rightMotor.setIdleMode(IdleMode.kBrake);
    initialize();
  }
  private void initialize(){
    RobotContainer.subsystems.add(this);
  }
  // public double getRotations(){
    // double leftRotations = leftMotor.getEncoder().getPosition();
    // double rightRotations = rightMotor.getEncoder().getPosition();
    // double avg = (Math.abs(rightRotations) + Math.abs(leftRotations)) /2.0;
    // return avg;
  // }
  public void setVoltage(double _volts){


    if(_volts >= 0){
     
    }else {

    }
    // leftMotor.setVoltage(_volts);
    // rightMotor.setVoltage(-_volts);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
