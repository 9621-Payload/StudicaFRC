// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TankDriveConstants;

public class Intake extends SubsystemBase {
  /* Give the subsystem the motors it needs */
  CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MotorId, MotorType.kBrushless);

  /* Creates a new DriveTrain. */
  public Intake() {

    intakeMotor.setSmartCurrentLimit(10, 20);

  }

  public void pull() {
    intakeMotor.set(1);
  }

  public void push() {
    intakeMotor.set(-1);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
