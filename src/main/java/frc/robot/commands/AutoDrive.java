// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  /** Creates a new Drive. */
  CommandSwerveDrivetrain driveSubsystem;
  double xSpeed, zRotation;
  private final SwerveRequest.RobotCentric autoRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoDrive(CommandSwerveDrivetrain driveSystem, double xSpeed, double zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    driveSubsystem.applyRequest(() ->
        autoRequest.withVelocityX(xSpeed)   // m/s forward
                   .withVelocityY(0)
                   .withRotationalRate(zRotation)
    ).withTimeout(2.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.applyRequest(() ->
        autoRequest.withVelocityX(0)   // m/s forward
                   .withVelocityY(0)
                   .withRotationalRate(0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
