// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.BooleanSupplier;

/** Commands for driver assistance features like auto-alignment */
public class DriverAssistCommands {

  // Standard offsets for alignment
  private static final double REEF_DISTANCE_OFFSET = Units.inchesToMeters(6.5);
  private static final double CORAL_DISTANCE_OFFSET = Units.inchesToMeters(4);
  private static final double ONE_INCH = Units.inchesToMeters(1);

  // Path constraints for different alignment scenarios
  private static final PathConstraints REEF_ALIGNMENT_CONSTRAINTS =
      new PathConstraints(
          3.0, // Max velocity in m/s
          4.0, // Max acceleration in m/s^2
          3.0, // Max angular velocity in rad/s
          4.0); // Max angular acceleration in rad/s^2

  private static final PathConstraints CORAL_ALIGNMENT_CONSTRAINTS =
      new PathConstraints(
          3.0, // Max velocity in m/s
          4.0, // Max acceleration in m/s^2
          3.0, // Max angular velocity in rad/s
          4.0); // Max angular acceleration in rad/s^2

  /**
   * Creates a command to align the robot to a reef tag position.
   *
   * @param drive The drive subsystem
   * @param vision The vision subsystem
   * @param alignRight True to align to the right of the tag, false for left
   * @param cancelSupplier A supplier that returns true when the command should be cancelled
   * @return The alignment command
   */
  public static Command alignToReefTag(
      Drive drive, Vision vision, boolean alignRight, BooleanSupplier cancelSupplier) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  Pose2d targetPose =
                      drive.calculateTagOffset(
                          vision.getClosestTagPose(0, drive.getPose()),
                          REEF_DISTANCE_OFFSET,
                          ONE_INCH * 3,
                          alignRight,
                          true);
                  if (targetPose != null) {
                    AutoBuilder.pathfindToPose(targetPose, REEF_ALIGNMENT_CONSTRAINTS)
                        .until(cancelSupplier)
                        .schedule();
                  }
                }),
            Commands.waitSeconds(0.1))
        .withName("Auto Align Reef " + (alignRight ? "Right" : "Left"));
  }

  /**
   * Creates a command to align the robot to a coral station tag.
   *
   * @param drive The drive subsystem
   * @param vision The vision subsystem
   * @param cancelSupplier A supplier that returns true when the command should be cancelled
   * @return The alignment command
   */
  public static Command alignToCoralTag(
      Drive drive, Vision vision, BooleanSupplier cancelSupplier) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  Pose2d targetPose =
                      drive.calculateTagOffset(
                          vision.getClosestTagPose(1, drive.getPose()),
                          0,
                          CORAL_DISTANCE_OFFSET,
                          false,
                          false);

                  if (targetPose != null) {
                    AutoBuilder.pathfindToPose(targetPose, CORAL_ALIGNMENT_CONSTRAINTS)
                        .until(cancelSupplier)
                        .schedule();
                  }
                }),
            Commands.waitSeconds(0.1))
        .withName("Auto Align Coral Station");
  }
}
