// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain m_drive;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  public DriveWithJoysticks(Drivetrain dt, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    m_drive = dt;
    m_left = leftSpeed;
    m_right = rightSpeed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(m_left.getAsDouble(), m_right.getAsDouble());
    SmartDashboard.putNumber("Left Speed", m_left.getAsDouble());
    SmartDashboard.putNumber("Right Speed", m_right.getAsDouble());
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
