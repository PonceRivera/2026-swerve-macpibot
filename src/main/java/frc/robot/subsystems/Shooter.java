package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase {

    public static final int MOTOR_LEFT_CAN_ID = 23;
    public static final int MOTOR_RIGHT_CAN_ID = 16;
    public static final double SHOOT_SPEED = 0.6;
    public static final double SHOOT_SPEED_RIGTH = -0.7;
    public static final double SHOOT_FULL_SPEED = 1.0;

    private static final double SPIN_UP_DELAY = 0.7;
    private static final double SHOOT_FULL_TIMEOUT = 4.0;

    private static final double DISTANCIA_MIN_METROS = 1.0;
    private static final double POTENCIA_MINima = 0.4;
    private static final double POTENCIA_MAXima = 1.0;
    private static final double PENDIENTE_POTENCIA = 0.15;

    private final SparkMax motorRight = new SparkMax(MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
    private final Timer spinUpTimer = new Timer();

    private final TalonFX motorLeft;
    private final TalonFXConfiguration leftConfig;

    private final DutyCycleOut m_leftRequest;

    public Shooter() {
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.smartCurrentLimit(60);
        motorRight.configure(rightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // this.motorLeft = new TalonFX(MOTOR_LEFT_CAN_ID, canbus);
        this.motorLeft = new TalonFX(23, TunerConstants.kCANBus.getName());
        this.leftConfig = new TalonFXConfiguration();

        // Configuración de motorLeft (Kraken)
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        this.m_leftRequest = new DutyCycleOut(0.50);
    }

    public void shoot() {
        motorLeft.setControl(m_leftRequest.withOutput(SHOOT_SPEED));
        motorRight.set(SHOOT_SPEED_RIGTH);
    }

    public void shootFull() {
        motorLeft.setControl(m_leftRequest.withOutput(SHOOT_FULL_SPEED));
        motorRight.set(-SHOOT_FULL_SPEED);
    }

    public void stop() {
        motorLeft.setControl(m_leftRequest.withOutput(0));
        motorRight.set(0);
    }

    public void dispararSegunDistancia(double distanciaMetros) {
        if (distanciaMetros <= 0) {
            stop();
            return;
        }

        double velocidadIzq = POTENCIA_MINima + (distanciaMetros - DISTANCIA_MIN_METROS) * PENDIENTE_POTENCIA + 0.02;
        velocidadIzq = Math.min(Math.max(velocidadIzq, POTENCIA_MINima), POTENCIA_MAXima);

        double ratioSpin = SHOOT_SPEED_RIGTH / SHOOT_SPEED;
        double velocidadDer = velocidadIzq * ratioSpin;

        motorLeft.setControl(m_leftRequest.withOutput(velocidadIzq));
        motorRight.set(velocidadDer);
    }

    @Override
    public void periodic() {
    }

    public Command shootCommand() {
        return Commands.runEnd(
                () -> {
                    motorRight.set(SHOOT_SPEED_RIGTH);
                    motorLeft.setControl(
                            m_leftRequest.withOutput(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? SHOOT_SPEED : 0.0));
                },
                () -> {
                    stop();
                    spinUpTimer.stop();
                },
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    public Command shootFullCommand() {
        return Commands.runEnd(
                () -> {
                    motorRight.set(-SHOOT_FULL_SPEED);
                    motorLeft.setControl(
                            m_leftRequest.withOutput(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? SHOOT_FULL_SPEED : 0.0));
                },
                () -> {
                    stop();
                    spinUpTimer.stop();
                },
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    public Command dispararSegunDistanciaCommand(DoubleSupplier distanceSupplier) {
        return Commands.runEnd(
                () -> {
                    double dist = distanceSupplier.getAsDouble();
                    if (dist <= 0) {
                        stop();
                        return;
                    }

                    double velocidadIzq = POTENCIA_MINima + (dist - DISTANCIA_MIN_METROS) * PENDIENTE_POTENCIA + 0.02;
                    velocidadIzq = Math.min(Math.max(velocidadIzq, POTENCIA_MINima), POTENCIA_MAXima);

                    double ratioSpin = SHOOT_SPEED_RIGTH / SHOOT_SPEED;
                    double velocidadDer = velocidadIzq * ratioSpin;

                    motorRight.set(velocidadDer);
                    motorLeft.setControl(
                            m_leftRequest.withOutput(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? velocidadIzq : 0.0));
                },
                this::stop,
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    public Command ShootStop() {
        return Commands.run(this::stop, this);
    }
}
