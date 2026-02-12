package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.LeftMotorId;
import static frc.robot.Constants.ShooterConstants.RightMotorId;
import static frc.robot.Constants.ShooterConstants.ShooterGains;
import static frc.robot.Constants.ShooterConstants.rps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motorRight = new TalonFX(RightMotorId);
    private final TalonFX motorLeft = new TalonFX(LeftMotorId);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = ShooterGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorRight.getConfigurator().apply(m_motorConfig);
        motorLeft.getConfigurator().apply(m_motorConfig);

        motorLeft.setControl(new Follower(RightMotorId, MotorAlignmentValue.Opposed));
    }

    // Sets the power of both motors.
    //
    // The left motor is set to the given power, while the right motor is set to the
    // negative of that power to ensure they spin in opposite directions.
    public void setPower(double power) {
        // motorRight.setControl(new DutyCycleOut(power));
        motorRight.setControl(m_request.withVelocity(rps));
    }

    // Disables both motors by setting their power to 0.
    public void stop() {
        motorRight.setControl(m_request.withVelocity(0));
    }
}
