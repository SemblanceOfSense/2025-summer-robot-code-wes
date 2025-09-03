package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;

public class IntakeSubsystem extends SubsystemBase {
    private final ConfigManager config = ConfigManager.getInstance();
    private final NetworkTablesUtils table = NetworkTablesUtils.getTable("debug");

    private final SparkFlex armMotor = new SparkFlex((int) config.get("ArmMotorId", IntakeConstants.armMotorId), MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final SparkFlex clawMotor = new SparkFlex((int) config.get("ClawMotorId", IntakeConstants.clawMotorId), MotorType.kBrushless);
    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    private final PIDController armPidController = new PIDController(
        config.get("ArmP", IntakeConstants.armP),
        config.get("ArmI", IntakeConstants.armI),
        config.get("ArmD", IntakeConstants.armD)
    );
    private final PIDController clawPidController = new PIDController(
        config.get("ClawP", IntakeConstants.clawP),
        config.get("ClawI", IntakeConstants.clawI),
        config.get("ClawD", IntakeConstants.clawD)
    );

    @Override
    public void periodic() {
        // Update Arm PID, position based
        armPidController.setSetpoint(table.getEntry("ArmSetpoint", 0));
        armMotor.setVoltage(config.get("ArmMaxVolt", IntakeConstants.armMaxVolt) * armPidController.calculate(MathUtils.RPMtoRadians(armEncoder.getPosition())));

        // Update Claw PID, angular velocity based
        clawPidController.setSetpoint(table.getEntry("ClawSetpoint", 0));
        clawMotor.set(config.get("ClawMaxAngVel", IntakeConstants.clawMaxAngVel) * clawPidController.calculate(MathUtils.RPMtoRadians(clawEncoder.getVelocity())));
    }

    public IntakeSubsystem() {}

    public void setArmPos(double pos) {
        table.setEntry("ArmSetpoint", pos);
    }

    public void setClawVel(double vel) {
        table.setEntry("ClawSetpoint", vel);
    }

    public void toggleArm() {
        if (table.getEntry("ArmSetPoint", 0) == 0) {
            table.setEntry("ArmSetPoint", config.get("ArmTargPos", IntakeConstants.armTargPos));
        } else {
            table.setEntry("ArmSetPoint", config.get("ArmTargPos", 0));
        }
    }

    public void toggleClaw() {
        if (table.getEntry("ClawSetPoint", 0) == 0) {
            table.setEntry("ClawSetPoint", config.get("ClawTargPos", IntakeConstants.clawTargVel));
        } else {
            table.setEntry("ClawSetPoint", config.get("ClawTargPos", 0));
        }
    }
}
