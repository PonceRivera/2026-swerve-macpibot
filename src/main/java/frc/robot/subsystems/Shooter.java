// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // --- Constants ---
    public static final int MOTOR_LEFT_CAN_ID = 16;
    public static final int MOTOR_RIGHT_CAN_ID = 17;
    public static final double SHOOT_SPEED = -0.8; // Teleop
    public static final double SHOOT_SPEED_RIGTH = -0.7; // Teleop
    public static final double SHOOT_FULL_SPEED = 1.0;

    private final SparkMax motorLeft = new SparkMax(MOTOR_LEFT_CAN_ID, MotorType.kBrushless);
    private final SparkMax motorRight = new SparkMax(MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
    private final InterpolatingDoubleTreeMap mapVelocidadPorDistancia = new InterpolatingDoubleTreeMap();
    private final Timer spinUpTimer = new Timer();

    public Shooter() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kCoast);
        leftConfig.smartCurrentLimit(60);
        motorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.smartCurrentLimit(60);
        rightConfig.inverted(true);
        motorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mapVelocidadPorDistancia.put(1.5, -0.4); // A 1.5 metros, pon el motor a -0.4
        mapVelocidadPorDistancia.put(2.5, -0.6); // A 2.5 metros, pon el motor a -0.6
        mapVelocidadPorDistancia.put(3.5, -0.8); // A 3.5 metros, pon el motor a -0.8
        mapVelocidadPorDistancia.put(4.5, -1.0); // A 4.5 metros, pon el motor a -1.0
    }

    // --- Public methods ---

    /** Dispara a velocidad de teleop (0.7). */
    public void shoot() {
        motorLeft.set(SHOOT_SPEED);
        motorRight.set(SHOOT_SPEED_RIGTH);
    }

    /** Dispara a potencia máxima (1.0) — usado en auto. */
    public void shootFull() {
        motorLeft.set(SHOOT_FULL_SPEED);
        motorRight.set(SHOOT_FULL_SPEED);
    }

    /** Detiene ambos motores del shooter. */
    public void stop() {
        motorLeft.set(0);
        motorRight.set(0);
    }

    /**
     * Calcula y aplica la velocidad necesaria basándose en la distancia de la
     * Limelight.
     * 
     * @param distanciaMetros - La distancia calculada usando Trigonometría.
     */
    public void dispararSegunDistancia(double distanciaMetros) {
        if (distanciaMetros <= 0) {
            stop(); // Si Limelight no ve nada o tira error de distancia, parar.
            return;
        }

        double velocidadRequeridaIzq = mapVelocidadPorDistancia.get(distanciaMetros);
        double velocidadRequeridaDer = velocidadRequeridaIzq * (SHOOT_SPEED_RIGTH / SHOOT_SPEED);

        motorLeft.set(velocidadRequeridaIzq);
        motorRight.set(velocidadRequeridaDer);
    }

    @Override
    public void periodic() {
        // Telemetría adicional puede añadirse aquí
    }

    // --- Command factories ---

    /**
     * Mientras se mantiene: el id 17 inicia, el id 16 se espera 2 segundos y luego
     * inician su rutina. Al soltar: paran los motores.
     */
    public Command shootCommand() {
        return Commands.runEnd(() -> {
            motorRight.set(SHOOT_SPEED_RIGTH);

            if (spinUpTimer.hasElapsed(0.5)) {
                motorLeft.set(SHOOT_SPEED);
            } else {
                motorLeft.set(0.0);
            }
        }, () -> {
            stop();
            spinUpTimer.stop();
        }, this).beforeStarting(() -> {
            spinUpTimer.reset();
            spinUpTimer.start();
        });
    }

    /** Auto: dispara a 1.0 durante 4 segundos y luego para solo. */
    public Command shootFullCommand() {
        return Commands.run(() -> this.shootFull(), this)
                .withTimeout(4)
                .andThen(Commands.runOnce(this::stop, this));
    }
}
