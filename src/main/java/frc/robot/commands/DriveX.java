// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveX extends CommandBase {
  private static final double TOLERANCE = 0.100;
  private static final double DRIVE_SPEED = 0.4;
  Drivetrain m_drivetrain;
  double m_delta;
  double m_destination;

  /** Creates a new DriveX. */
  public DriveX(double x) {
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);
    m_delta = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_destination = m_drivetrain.getPose2d().getX() + m_delta;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_vel = DRIVE_SPEED * Math.signum(dx());
    m_drivetrain.drive(new Translation2d(x_vel, 0), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(dx()) < TOLERANCE;
  }

  public double dx(){
    return m_destination - m_drivetrain.getPose2d().getX();
  }
}
