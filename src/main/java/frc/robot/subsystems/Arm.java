// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  WPI_VictorSPX armMotor = new WPI_VictorSPX(Constants.ArmConstants.MotorId);

  /**
   * Encoder operates in degrees
   */
  Encoder armEncoder = new Encoder(
      Constants.ArmConstants.EncChannelA,
      Constants.ArmConstants.EncChannelB);

  PIDController armPlant = new PIDController(0.15, 0.005, 0);

  double targetAngleDeg = 0,
      currentAngleDeg = 0;

  public Arm() {

    armMotor.setInverted(true);

    armEncoder.setDistancePerPulse(
        360 /
            Constants.ArmConstants.EncPPR /
            Constants.ArmConstants.gearRatio);

    armPlant.setTolerance(Constants.ArmConstants.positionToleranceAngle);

    setTargetAngle(armEncoder.getDistance());
  }

  public void setTargetAngle(double targetAngle) {
    targetAngleDeg = targetAngle;
  }

  public boolean isAtTarget (){
    return armPlant.atSetpoint();
  }


  @Override
  public void periodic() {

    currentAngleDeg = armEncoder.getDistance();

    armMotor.set(armPlant.calculate(currentAngleDeg, targetAngleDeg));

    SmartDashboard.putNumber("Arm angle", currentAngleDeg);
    SmartDashboard.putNumber("Target angle", targetAngleDeg);
    SmartDashboard.putBoolean("Arm at target", isAtTarget());
    SmartDashboard.putNumber("Arm power", armMotor.get());
  }

}
