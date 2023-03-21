// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveOntoRamp extends CommandBase {
  private static final double PITCH_THRESHOLD = 8.0;
  private static final double DRIVE_SPEED_PCT = 0.3;
  private Drivetrain m_drivetrain;

  /** Creates a new DriveOntoRamp. */
  public DriveOntoRamp(Drivetrain subsystem) {
    m_drivetrain = subsystem;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.driveHeading(new Translation2d(DRIVE_SPEED_PCT, Rotation2d.fromDegrees(0)), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("pitch is: " + m_drivetrain.getPitchDeg());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getPitchDeg()) > PITCH_THRESHOLD;
  }
}
