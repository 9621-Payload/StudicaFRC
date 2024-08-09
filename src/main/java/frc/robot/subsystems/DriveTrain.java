// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankDriveConstants;

public class DriveTrain extends SubsystemBase {
  /* Give the subsystem the motors it needs */
  CANSparkMax rightFront = new CANSparkMax(TankDriveConstants.frontRightId, MotorType.kBrushless);  
  CANSparkMax rightBack = new CANSparkMax(TankDriveConstants.backRightId, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(TankDriveConstants.backLeftId, MotorType.kBrushless);
  CANSparkMax leftFront = new CANSparkMax(TankDriveConstants.frontLeftId, MotorType.kBrushless);

  DifferentialDrive tankDriveSystem = new DifferentialDrive(leftFront, rightFront);

  /* Creates a new DriveTrain. */
  public DriveTrain() {
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
  }

  public DifferentialDrive getTankDrive() {
    return tankDriveSystem;
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
