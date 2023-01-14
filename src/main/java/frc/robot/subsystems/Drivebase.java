// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C.Port;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */
  public Drivebase() {
    m_rightMaster.setInverted(true);
  }

  CANSparkMax m_leftMotor1 = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_leftMotor2 = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_leftMotor3 = new CANSparkMax(1, MotorType.kBrushed);

  CANSparkMax m_rightMotor1 = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_rightMotor2 = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_rightMotor3 = new CANSparkMax(1, MotorType.kBrushed);

  MotorControllerGroup m_leftMaster = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
  MotorControllerGroup m_rightMaster = new MotorControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

  

  DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);


  AHRS m_gyro = new AHRS(Port.kMXP);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public CommandBase Foward() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          arcadeDrive(1, 0);
        });
  }

  public CommandBase getZAngle() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return boolean run(
        () -> {
          return m_gryo.getAngle();
        });
  }

  public CommandBase callibrate() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return void runOnce(
        () -> {
          m_gryo.callibrate();
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public double getAngle(){
    return m_gyro.getAngle();
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    m_diffDrive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    SmartDashboard.puNumber("Z ANfgle", m_gyro.getAngle());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
