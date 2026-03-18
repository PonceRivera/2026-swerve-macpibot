package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cameraserver.CameraServer;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tid;

    // --- CONSTANTES PARA CALCULAR DISTANCIA ---
    // Altura desde el piso hasta el lente de la cámara (en metros)
    private static final double ALTURA_LENTE_METROS = 0.50;
    // Altura desde el piso hasta el centro del AprilTag que quieres disparar (en
    // metros)
    private static final double ALTURA_OBJETIVO_METROS = 1.45;
    // Ángulo de inclinación de la cámara (hacia arriba) respecto al piso (en
    private static final double ANGULO_MONTAJE_GRADOS = 30.0;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        // El ID del AprilTag principal que está viendo (por ejemplo, 1, 2, 3...)
        tid = table.getEntry("tid");

        // Enviar la imagen de la Limelight a Shuffleboard usando HttpCamera
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

    /**
     * @return true si la cámara está detectando un objetivo/AprilTag
     */
    public boolean tieneObjetivo() {
        return tv.getDouble(0.0) == 1.0;
    }

    /**
     * @return
     */
    public int getTagID() {
        if (tieneObjetivo()) {
            return (int) tid.getDouble(-1.0);
        }
        return -1;
    }

    /**
     * @return
     */
    public double getXOffset() {
        return tx.getDouble(0.0);
    }

    /**
     * @return El ángulo vertical hacia el objetivo (grados)
     */
    public double getYOffset() {
        return ty.getDouble(0.0);
    }

    /**
     * @return La distancia en metros (0.0 si no detecta nada).
     */
    public double getDistanciaMetros() {
        if (!tieneObjetivo())
            return 0.0;

        // ty es el ángulo vertical de la cámara al centro del objetivo (el AprilTag)
        double anguloHaciaObjetivoRadianes = Math.toRadians(ANGULO_MONTAJE_GRADOS + getYOffset());
        double diferenciaAltura = ALTURA_OBJETIVO_METROS - ALTURA_LENTE_METROS;

        // d = (h2 - h1) / tan(a1 + a2)
        return diferenciaAltura / Math.tan(anguloHaciaObjetivoRadianes);
    }

    @Override
    public void periodic() {
        // Imprimir los datos del Limelight en el SmartDashboard para que los veas
        SmartDashboard.putBoolean("Limelight/Tiene Objetivo", tieneObjetivo());
        SmartDashboard.putNumber("Limelight/ID del AprilTag", getTagID());
        SmartDashboard.putNumber("Limelight/Angulo X", getXOffset());
        SmartDashboard.putNumber("Limelight/Angulo Y", getYOffset());
        SmartDashboard.putNumber("Limelight/Distancia (m)", getDistanciaMetros());
    }
}