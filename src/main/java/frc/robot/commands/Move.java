// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TankDriveConstants;

/** An example command that uses an example subsystem. */
public class Move extends Command {
  private final DriveTrain mDriveTrain;
  private DoubleSupplier mSpeed;
  private DoubleSupplier mRotate;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Move(DriveTrain subsystem, DoubleSupplier speed, DoubleSupplier rotate) {
    mDriveTrain = subsystem;
    mSpeed = speed;
    mRotate = rotate;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveTrain.getTankDrive().arcadeDrive(mSpeed.getAsDouble() * TankDriveConstants.kSpeedFactor, mRotate.getAsDouble() * TankDriveConstants.kSpeedFactor);
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
