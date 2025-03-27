package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lift.Lift.LiftPosition;
import org.littletonrobotics.junction.Logger;

/** Commands for safely operating the lift with coral. */
public class LiftCommands {

  /**
   * Creates a command that safely moves the lift to a position after ensuring coral safety.
   * Prevents coral damage by checking sensors and positioning before lift movement.
   *
   * @param endEffector The end effector subsystem
   * @param lift The lift subsystem
   * @param targetPosition The desired lift position
   * @return A command that safely moves the lift only when coral is properly positioned
   */
  public static Command safeCoralLift(
      EndEffector endEffector, Lift lift, LiftPosition targetPosition) {
    return Commands.runOnce(() -> lift.runLiftToPos(targetPosition));
    // return safeCoralLift(endEffector, lift, targetPosition.getPosition());
  }

  /**
   * Creates a command that safely moves the lift to a specific height after ensuring coral safety.
   * Prevents coral damage by checking sensors and positioning before lift movement.
   *
   * @param endEffector The end effector subsystem
   * @param lift The lift subsystem
   * @param targetPositionInches The desired lift position in inches
   * @return A command that safely moves the lift only when coral is properly positioned
   */
  public static Command safeCoralLift(
      EndEffector endEffector, Lift lift, double targetPositionInches) {

    // Command that checks coral position and moves lift only when safe
    return new Command() {
      private final Timer safetyTimer = new Timer();
      private boolean liftMovementStarted = false;
      private boolean coralSafetyAlert = false;
      private static final double SAFETY_TIMEOUT = 3.0; // Time to wait for safety before alerting
      private boolean isMovingUp;
      private boolean algaeMechanismPositioned = false;

      @Override
      public void initialize() {
        Logger.recordOutput("Commands/SafeCoralLift", "Starting safety checks");

        // Determine if we're moving up or down
        double currentPosition = lift.getCurrentPosition();
        isMovingUp = targetPositionInches > currentPosition;
        Logger.recordOutput("Commands/SafeCoralLift/IsMovingUp", isMovingUp);

        // Position algae mechanism based on direction
        if (isMovingUp) {
          Logger.recordOutput(
              "Commands/SafeCoralLift", "Moving up - setting algae mech to ready position");
          endEffector.moveToReadyPos(); // Ready position keeps it out of the way when moving up
        } else {
          Logger.recordOutput(
              "Commands/SafeCoralLift", "Moving down - setting algae mech to stow position");
          endEffector.moveToStow(); // Stow position is safer when moving down
        }

        safetyTimer.reset();
        safetyTimer.start();
        liftMovementStarted = false;
        coralSafetyAlert = false;
        algaeMechanismPositioned = false;
      }

      @Override
      public void execute() {
        // Check if algae mechanism has had time to get into position
        if (!algaeMechanismPositioned) {
          // Give it a short time to position before proceeding
          if (safetyTimer.hasElapsed(0.2)) {
            algaeMechanismPositioned = true;
            Logger.recordOutput("Commands/SafeCoralLift", "Algae mechanism positioned");
          } else {
            return; // Wait for algae mechanism to position
          }
        }

        // Check if coral is in a safe position using sensor inputs
        // Only the back sensor matters for lift safety
        boolean backSensorClear = !endEffector.isBackSensorTriggered();
        boolean coralPositionSafe = isCoralPositionSafe(endEffector);

        // Log safety conditions
        Logger.recordOutput("Commands/SafeCoralLift/BackSensorClear", backSensorClear);
        Logger.recordOutput("Commands/SafeCoralLift/CoralPositionSafe", coralPositionSafe);

        // If safety conditions are met, proceed with lift movement
        // Front sensor is no longer considered as it's not relevant for lift safety
        if (coralPositionSafe && backSensorClear) {
          if (!liftMovementStarted) {
            Logger.recordOutput("Commands/SafeCoralLift", "Coral position safe, moving lift");
            lift.runLiftToPos(targetPositionInches);
            liftMovementStarted = true;
          }
        }
        // Otherwise, display an alert if we've been waiting too long
        else if (safetyTimer.hasElapsed(SAFETY_TIMEOUT) && !coralSafetyAlert) {
          Logger.recordOutput("Commands/SafeCoralLift", "ALERT: Coral not in safe position");
          // Could trigger additional alerts, lights, or sounds here
          coralSafetyAlert = true;
        }
      }

      @Override
      public boolean isFinished() {
        // Only finish when:
        // 1. Lift movement has started
        // 2. Lift has reached target position
        return liftMovementStarted && lift.isAtTargetPosition();
      }

      @Override
      public void end(boolean interrupted) {
        safetyTimer.stop();
        Logger.recordOutput(
            "Commands/SafeCoralLift",
            "Ended - " + (interrupted ? "Interrupted" : "Position reached safely"));
      }
    }.withName("SafeCoralLift").withTimeout(10.0); // Overall safety timeout
  }

  /**
   * Checks if the coral is in a safe position using various sensors and measurements. This method
   * combines multiple safety checks to ensure the coral won't be damaged.
   *
   * @param endEffector The end effector with coral position sensors
   * @return true if coral is safely positioned, false otherwise
   */
  private static boolean isCoralPositionSafe(EndEffector endEffector) {
    // Focus only on the back sensor for lift safety

    // 1. Check if coral is completely clear of lift mechanism
    boolean coralClearOfLift = !endEffector.isBackSensorTriggered();

    // 2. Check if algae removal mechanism is in stowed position
    boolean algaeRemoverStowed = endEffector.getAlgaePosition() > -10.0;

    // Return true only if ALL safety conditions are met
    return coralClearOfLift && algaeRemoverStowed;
  }

  /**
   * Creates a command sequence that safely positions coral before attempting to move the lift. This
   * can be used when coral needs to be moved to a safe position first.
   *
   * @param endEffector The end effector subsystem
   * @param lift The lift subsystem
   * @param targetPosition The desired final lift position
   * @return A command sequence that positions coral safely then moves lift
   */
  public static Command positionCoralAndLift(
      EndEffector endEffector, Lift lift, LiftPosition targetPosition) {

    // Determine if we're moving up or down for proper algae mechanism positioning
    double currentPosition = lift.getCurrentPosition();
    boolean movingUp = targetPosition.getPosition() > currentPosition;

    return Commands.sequence(
            Commands.runOnce(() -> endEffector.stop()),
            // First make sure coral is in safe position
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "Commands/PositionCoralAndLift", "Starting coral positioning")),

            // Position algae mechanism based on lift direction
            Commands.runOnce(
                () -> {
                  if (movingUp) {
                    endEffector.moveToReadyPos(); // Ready position when moving up
                  } else {
                    endEffector.moveToStow(); // Stow position when moving down
                  }
                }),

            // Allow short time for algae mechanism to move to position
            Commands.waitSeconds(0.2),

            // Move coral to safe position
            // IntakeCommands.prepareForScoring(endEffector),

            // Then move lift when safe
            safeCoralLift(endEffector, lift, targetPosition))
        // Commands.runOnce(() -> lift.runLiftToPos(targetPosition)))
        .withName("PositionCoralAndLift");
  }

  /**
   * Creates an emergency command to safely handle a situation where coral may be in danger. This
   * command can be bound to a button for quick response to potential damage scenarios.
   *
   * @param endEffector The end effector subsystem
   * @param lift The lift subsystem
   * @return An emergency command that moves everything to a safe position
   */
  public static Command emergencyCoralSafety(EndEffector endEffector, Lift lift) {
    return Commands.sequence(
            // Log the emergency
            Commands.runOnce(
                () ->
                    Logger.recordOutput(
                        "Commands/EmergencyCoralSafety", "EMERGENCY SAFETY ACTIVATED")),

            // Move lift to bottom position (safest position)
            Commands.runOnce(() -> lift.runLiftToPos(Lift.LiftPosition.BOTTOM)),

            // Move coral to safe position
            IntakeCommands.prepareForScoring(endEffector),

            // Wait for everything to reach safe positions - only check back sensor
            Commands.waitUntil(
                () -> lift.isAtTargetPosition() && !endEffector.isBackSensorTriggered()))
        .withName("EmergencyCoralSafety")
        .withTimeout(5.0); // Safety timeout
  }
}
