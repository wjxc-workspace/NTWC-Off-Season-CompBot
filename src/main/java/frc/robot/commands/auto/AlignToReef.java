// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.vision.LimelightHelpers;
import frc.robot.AutoScoreManager.ScorePosition;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;

public class AlignToReef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Swerve swerve;
  private double tagId = -1;
  double xSpeed, ySpeed, rotSpeed;

  public AlignToReef(Swerve swerve, ScorePosition position) {
    this.swerve = swerve;
    this.isRightScore = position.isRightScore();
    this.tagId = position.getId();
    xController = new PIDController(AutoScoreConstants.kDistanceP, 0.0, 0);
    yController = new PIDController(AutoScoreConstants.kDistanceP, 0.0, 0);
    rotController = new PIDController(AutoScoreConstants.kRotationP, 0, 0);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    stopTimer = new Timer();
    stopTimer.start();

    dontSeeTagTimer = new Timer();
    dontSeeTagTimer.start();

    rotController.setSetpoint(0);
    rotController.setTolerance(1);

    // minus since the robot should be at the -x axis of the tag
    xController.setSetpoint(-AutoScoreConstants.kDistanceToPreReef);
    xController.setTolerance(0.02);

    // the +- 0.02 is just for testing - our wooden field isn't correct
    yController.setSetpoint(isRightScore ? AutoScoreConstants.kLeftReefPosition : -AutoScoreConstants.kLeftReefPosition);
    yController.setTolerance(0.02);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getFiducialID(VisionConstants.kFrontCameraName) == tagId
        || LimelightHelpers.getFiducialID(VisionConstants.kLeftCameraName) == tagId
        || LimelightHelpers.getFiducialID(VisionConstants.kRightCameraName) == tagId) {
      System.out.println(LimelightHelpers.getFiducialID(VisionConstants.kFrontCameraName));
      System.out.println(LimelightHelpers.getFiducialID(VisionConstants.kLeftCameraName));
      System.out.println(LimelightHelpers.getFiducialID(VisionConstants.kRightCameraName));

      dontSeeTagTimer.reset();
      if (isRightScore) {
        if (LimelightHelpers.getFiducialID(VisionConstants.kLeftCameraName) == tagId) {
          double[] postions = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.kLeftCameraName);
          xSpeed = xController.calculate(postions[2], -AutoScoreConstants.kDistanceToReef);
          ySpeed = -yController.calculate(postions[0]);
          rotSpeed = -rotController.calculate(postions[4]);
        } else {
          double[] postions = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.kFrontCameraName);
          xSpeed = xController.calculate(postions[2]);
          ySpeed = -yController.calculate(postions[0]);
          rotSpeed = -rotController.calculate(postions[4]);
        } 
      } else {
        if (LimelightHelpers.getFiducialID(VisionConstants.kRightCameraName) == tagId) {
          double[] postions = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.kRightCameraName);
          xSpeed = xController.calculate(postions[2], -AutoScoreConstants.kDistanceToReef);
          ySpeed = -yController.calculate(postions[0]);
          rotSpeed = -rotController.calculate(postions[4]);
        } else {
          double[] postions = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.kFrontCameraName);
          xSpeed = xController.calculate(postions[2]);
          ySpeed = -yController.calculate(postions[0]);
          rotSpeed = -rotController.calculate(postions[4]);
        } 
      }

      swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false);

      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      swerve.drive(new Translation2d(), 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    return dontSeeTagTimer.hasElapsed(0.1) || stopTimer.hasElapsed(0.3);
  }
}