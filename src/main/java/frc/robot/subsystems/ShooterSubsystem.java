package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.TunerConstants;

/**
 * ShooterSubsystem – controls the left and right Kraken X60 (TalonFX)
 * flywheel shooter motors using Phoenix 6 velocity closed-loop control.
 *
 * Motor IDs are read from {@link frc.robot.Constants.ShooterConstants}.
 *
 * Both wheels run at the same target speed. To support differential shooting
 * (different top/bottom speeds) add a second setpoint parameter or a second
 * setTarget method.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ---------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------
    private final TalonFX intakeLauncherRoller;  // left shooter motor
    private final TalonFX feederRoller; // right shooter motor

    // ---------------------------------------------------------------
    // Control requests  (reused every loop to avoid allocation)
    // ---------------------------------------------------------------
    /** Velocity request (RPS) with feed-forward enabled. */
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(false);   // set true if using Phoenix Pro

    private final VoltageOut m_voltageOut = new VoltageOut(0);

    /** Coast both motors (used when idle). */
    private final NeutralOut m_coastRequest = new NeutralOut();

    // ---------------------------------------------------------------
    // State
    // ---------------------------------------------------------------
    private double m_targetRPS = 0.0;

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    public ShooterSubsystem() {
        intakeLauncherRoller  = new TalonFX(ShooterConstants.kLeftShooterMotorId, "rio");
        feederRoller = new TalonFX(ShooterConstants.kRightShooterMotorId, "rio");
        configureMotors();
    }

    // ---------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------
    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Coast in neutral so flywheel can decelerate freely when not shooting
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Supply current limit to protect wiring
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = 100.0;

        // Velocity closed-loop gains (Slot 0)
        // kS: static friction overcome, kV: velocity feed-forward, kP: proportional
        // These starting values work for a typical flywheel – tune with SysId
        config.Slot0 = new Slot0Configs()
            .withKS(0.05)
            .withKV(0.12)
            .withKA(0.01)
            .withKP(0.11)
            .withKI(0.0)
            .withKD(0.0);

        // Left motor: NOT inverted (faces forward)
        TalonFXConfiguration leftConfig  = config;
        leftConfig.MotorOutput.Inverted  = InvertedValue.CounterClockwise_Positive;
        applyConfig(intakeLauncherRoller, leftConfig, "Left Shooter");

        // Right motor: INVERTED (faces opposite direction)
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits = config.CurrentLimits;
        rightConfig.Slot0         = config.Slot0;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        applyConfig(feederRoller, rightConfig, "Right Shooter");
    }

    private void applyConfig(TalonFX motor, TalonFXConfiguration cfg, String name) {
        StatusCode status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.err.println("[ShooterSubsystem] Failed to configure " + name + ": " + status);
        }
    }

    // ---------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------

    /**
     * Spin both flywheels to the given speed.
     *
     * @param targetRPS Target velocity in rotations per second (positive = shooting direction)
     */
    public void setSpeed(double intakeShootVolts, double feedVolts) {
        intakeLauncherRoller.setControl(m_voltageOut.withOutput(intakeShootVolts));
        feederRoller.setControl(m_voltageOut.withOutput(feedVolts));
    }

    // A method to set the voltage of the intake roller
    public void setIntakeLauncherRoller(double intakeShootVolts) {
        intakeLauncherRoller.setControl(m_voltageOut.withOutput(intakeShootVolts));
    }

    // A method to set the voltage of the intake roller
    public void setFeederRoller(double feedVolts) {
        feederRoller.setControl(m_voltageOut.withOutput(feedVolts));
    }

    /** Coast both flywheels (do not apply any output). */
    public void coast() {
        m_targetRPS = 0.0;
        intakeLauncherRoller .setControl(m_coastRequest);
        feederRoller.setControl(m_coastRequest);
    }

    public void stop() {
        coast();
    }

    public static boolean isHubActive() {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      // If we have no alliance, we cannot be enabled, therefore no hub.
      if (alliance.isEmpty()) {
        return false;
      }
      // Hub is always enabled in autonomous.
      if (DriverStation.isAutonomousEnabled()) {
        return true;
      }
      // At this point, if we're not teleop enabled, there is no hub.
      if (!DriverStation.isTeleopEnabled()) {
        return false;
      }

      // We're teleop enabled, compute.
      double matchTime = DriverStation.getMatchTime();
      String gameData = DriverStation.getGameSpecificMessage();
      // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
      if (gameData.isEmpty()) {
        return true;
      }
      boolean redInactiveFirst = false;
      switch (gameData.charAt(0)) {
        case 'R' -> redInactiveFirst = true;
        case 'B' -> redInactiveFirst = false;
        default -> {
          // If we have invalid game data, assume hub is active.
          return true;
        }
      }

      // Shift was is active for blue if red won auto, or red if blue won auto.
      boolean shift1Active = switch (alliance.get()) {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
      };

      if (matchTime > 130) {
        // Transition shift, hub is active.
        return true;
      } else if (matchTime > 105) {
        // Shift 1
        return shift1Active;
      } else if (matchTime > 80) {
        // Shift 2
        return !shift1Active;
      } else if (matchTime > 55) {
        // Shift 3
        return shift1Active;
      } else if (matchTime > 30) {
        // Shift 4
        return !shift1Active;
      } else {
        // End game, hub always active.
        return true;
      }
    }

    // ---------------------------------------------------------------
    // Command factories (inline command pattern)
    // ---------------------------------------------------------------

    /**
     * Returns a command that spins the shooter at the given RPS and never ends
     * (runs until cancelled / interrupted).
     */
    // public Command spinCommand(double targetRPS) {
    //     return runEnd(
    //         () -> setSpeed(targetRPS),
    //         this::coast
    //     );
    // }

    /** Coast the shooter (stop shooting). */
    public Command coastCommand() {
        return runOnce(this::coast);
    }

    // ---------------------------------------------------------------
    // Periodic
    // ---------------------------------------------------------------

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter/LeftVelocityRPS",
        //     intakeLauncherRoller.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/RightVelocityRPS",
        //     feederRoller.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Shooter/TargetRPS", m_targetRPS);
      SmartDashboard.putBoolean("Hub Active", isHubActive());
    }
}
