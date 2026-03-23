package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import frc.robot.LimelightHelpers;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cameraserver.CameraServer;

public class Limelight extends SubsystemBase {
    private static final double ALTURA_LENTE_METROS = Units.inchesToMeters(24.5);
    private static final double ALTURA_OBJETIVO_METROS = Units.inchesToMeters(52.75); // Altura al centro del AprilTag en el Hub (Rebuilt)
    private static final double ANGULO_MONTAJE_GRADOS = 0.0;     // Camara paralela al piso

    // Offset desde el lente de la cámara al centro del robot o disparador.
    // Eje X: Frente/Atrás (+X es el frente del robot). Eje Y: Izquierda/Derecha (+Y es la izquierda).
    private static final double OFFSET_X_CAMARA_AL_DISPARADOR_METROS = Units.inchesToMeters(-4.5); // 4.5 in hacia atrás
    private static final double OFFSET_Y_CAMARA_AL_DISPARADOR_METROS = Units.inchesToMeters(-9.0); // 9.0 in a la derecha


    public Limelight() {

        try {
            String[] urls = {
                    "http://limelight.local:5800/stream.mjpg",
                    "http://10.82.90.11:5800/stream.mjpg"
            };
            HttpCamera limelightFeed = new HttpCamera("Limelight", urls, HttpCameraKind.kMJPGStreamer);
            CameraServer.addCamera(limelightFeed);
            Shuffleboard.getTab("Limelight").add("Cámara", limelightFeed).withSize(4, 3);
        } catch (Exception e) {
            System.out.println("No se pudo agregar Limelight a Shuffleboard: " + e.getMessage());
        }
    }

    public boolean tieneObjetivo() {
        return LimelightHelpers.getTV("limelight");
    }

    public int getTagID() {
        return (int) LimelightHelpers.getFiducialID("limelight");
    }

    public double getXOffset() {
        return LimelightHelpers.getTX("limelight");
    }

    public double getYOffset() {
        return LimelightHelpers.getTY("limelight");
    }

    public double getDistanciaCamaraMetros() {
        if (!tieneObjetivo())
            return 0.0;

        double anguloHaciaObjetivoRadianes = Math.toRadians(ANGULO_MONTAJE_GRADOS + getYOffset());
        double diferenciaAltura = ALTURA_OBJETIVO_METROS - ALTURA_LENTE_METROS;

        return diferenciaAltura / Math.tan(anguloHaciaObjetivoRadianes);
    }

    public double getDistanciaMetros() {
        if (!tieneObjetivo())
            return 0.0;

        double distanciaCam = getDistanciaCamaraMetros();
        double txRadianes = Math.toRadians(getXOffset());
        
        // Coordenadas del objetivo relativo a la cámara (+X frente, +Y izquierda)
        // tx es la desviación angular horizontal; asumimos tx positivo = a la derecha.
        double targetX_Cam = distanciaCam * Math.cos(txRadianes);
        double targetY_Cam = distanciaCam * Math.sin(-txRadianes); // -Y es la derecha

        // Coordenadas del objetivo pero relativas al centro del disparador
        double deltaX = targetX_Cam - OFFSET_X_CAMARA_AL_DISPARADOR_METROS;
        double deltaY = targetY_Cam - OFFSET_Y_CAMARA_AL_DISPARADOR_METROS;

        // Distancia real desde la base del disparador al objetivo (teorema de Pitágoras/Hipotenusa)
        return Math.hypot(deltaX, deltaY);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/Tiene Objetivo", tieneObjetivo());
        SmartDashboard.putNumber("Limelight/ID del AprilTag", getTagID());
        SmartDashboard.putNumber("Limelight/Angulo X", getXOffset());
        SmartDashboard.putNumber("Limelight/Angulo Y", getYOffset());
        
        if (tieneObjetivo()) {
            SmartDashboard.putNumber("Limelight/Distancia Disparador (m)", getDistanciaMetros());
            SmartDashboard.putNumber("Limelight/Distancia Camara (m)", getDistanciaCamaraMetros());
        } else {
            SmartDashboard.putNumber("Limelight/Distancia Disparador (m)", 0.0);
            SmartDashboard.putNumber("Limelight/Distancia Camara (m)", 0.0);
        }
    }
}