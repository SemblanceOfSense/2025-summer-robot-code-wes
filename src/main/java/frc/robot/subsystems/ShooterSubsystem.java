package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final ConfigManager config = ConfigManager.getInstance();

    private final SparkFlex shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, SparkLowLevel.MotorType.kBrushless);

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
            config.get("ShooterFFKs", ShooterConstants.shooterFFKs),
            config.get("ShooterFFKv", ShooterConstants.shooterFFKv),
            config.get("ShooterFFKa", ShooterConstants.shooterFFKa)
    );

    private final PIDController shooterPID = new PIDController(
            config.get("ShooterP", ShooterConstants.shooterP),
            config.get("ShooterI", ShooterConstants.shooterI),
            config.get("ShooterD", ShooterConstants.shooterD)
    );

    public ShooterSubsystem() {

    }


    public void setMotorRPM(double rpm) {
        shooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(shooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterPID.calculate(MathUtils.RPMtoRadians(rpm))
        );
    }

    public void runVolts(double volts) {
        shooterMotor.setVoltage(volts);
    }

    public void resetPID() {
        shooterPID.reset();
    }
}