package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX motorSlapdown = new TalonFX(SlapdownMotorId);
    private final TalonFX motorIntake = new TalonFX(IntakeMotorId);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IntakeGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorSlapdown.getConfigurator().apply(m_motorConfig);
        motorIntake.getConfigurator().apply(m_motorConfig);
    }

    //Sets the power of intake motor.
    public void setPowerIntake(double power) {
        //rps prolly needs to be multipled with some constant
        motorIntake.setControl(m_request.withVelocity(rps));
    }

    //Sets the power of the slapdown motor.
    public void setPowerSlapdown(double power) {
        //rps prolly needs to be multiplied with some constant
        motorSlapdown.setControl(m_request.withVelocity(rps));
    }

    //Retracts the slapdown.
    public void retractSlapdown(double power) {
        //rps prolly needs to be multiplied with some constant
        motorSlapdown.setControl(m_request.withVelocity(-rps));
    }

    public void stopIntake() {
        motorIntake.setControl(m_request.withVelocity(0));
    }

    public void stopSlapdown() {
        motorIntake.setControl(m_request.withVelocity(0));
    }
}
