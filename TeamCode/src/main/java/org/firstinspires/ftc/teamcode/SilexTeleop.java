package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "Mipha", group = "ORION")
public class SilexTeleop extends LinearOpMode {

    // DRIVE
    private DcMotorEx MotorDelDer, MotorTraDer, MotorDelIzq, MotorTraIzq;

    // INTAKE
    private DcMotor Intake;

    // TURRET MOTOR - SEGUIMIENTO APRILTAG
    private DcMotorEx Turret;

    // SHOOTER
    private DcMotorEx ShooterIzq;
    private DcMotorEx ShooterDer;

    // GATE SERVO
    private Servo Gate;

    // TILT SERVOS
    private Servo TiltDer, TiltIzq;

    // TURRET SERVOS
    private Servo TurretLeftServo;
    private Servo TurretRightServo;
    private double turretLeftServoPosition;  // Posición actual Left
    private double turretRightServoPosition; // Posición actual Right (invertida)

    // TURRET SERVO CONFIGURACIÓN
    private double TURRET_SERVO_LEFT_MIN = 0.0;      // Tope inferior Left
    private double TURRET_SERVO_LEFT_MAX = 0.7;      // Tope superior Left
    private double TURRET_SERVO_LEFT_INIT = 0.7;     // Posición inicial Left
    private double TURRET_SERVO_RIGHT_MIN = 0.2;     // Tope inferior Right
    private double TURRET_SERVO_RIGHT_MAX = 0.7;     // Tope superior Right
    private double TURRET_SERVO_RIGHT_INIT = 0.2;    // Posición inicial Right
    private static final double TURRET_SERVO_STEP = 0.02; // Velocidad de movimiento

    // SENSOR DE COLOR PARA CONTAR PELOTAS
    private NormalizedColorSensor intakeSensor;

    // LIMELIGHT 3A - APRILTAG TRACKING
    private Limelight3A limelight;

    // ESTADO GENERAL
    private static final double DEADBAND = 0.05;

    // TILT STATE Y RANGOS
    private boolean tiltActive = false;
    private boolean tiltButtonPrev = false;

    // Posiciones independientes para el servo Derecho (TiltDer)
    private static final double TILT_DER_INITIAL_POS = 0.5;
    private static final double TILT_DER_FINAL_POS   = 0.83;

    // Posiciones independientes para el servo Izquierdo (TiltIzq)
    private static final double TILT_IZQ_INITIAL_POS = 0.5;
    private static final double TILT_IZQ_FINAL_POS   =0.07;

    // GATE POSITIONS
    private static final double GATE_CLOSED_POS = 1.5;  // Posición cerrada
    private static final double GATE_OPEN_POS = 0.7;    // Posición abierta
    private static final double GATE_DELAY_SECONDS = 2; // Delay antes de abrir el Gate

    // GATE TIMING
    private double shooterStartTime = 0;
    private boolean shooterActive = false;

    // ===================== CONTEO DE PELOTAS =====================
    private int pelotasContadas = 0;
    private boolean pelotaDetectadaAnteriormente = false;
    private static final double UMBRAL_DETECCION = 0.008; // Ajustar según pruebas

    // ===================== APRILTAG TRACKING - LIMELIGHT =====================
    private boolean aprilTagAutoAlign = false;
    private boolean aprilTagAlignButtonPrev = false;
    private static final double APRILTAG_ALIGN_KP = 0.03; // Ganancia para alineación
    private static final double APRILTAG_ALIGN_STEER_GAIN = 0.02; // Ganancia de giro suave

    // ===================== TURRET PID CONTROL =====================
    private double turretLastError = 0;
    private double turretSmoothedPower = 0; // Para suavizar la salida
    private static final double TURRET_KP = 0.008;   // Ganancia proporcional conservadora
    private static final double TURRET_KD = 0.004;  // Derivativa para amortiguar oscilaciones
    private static final double TURRET_MAX_POWER = 0.25; // Potencia máxima moderada
    private static final double TURRET_SMOOTHING = 0.3; // Factor de suavizado (0.0-1.0)

    // ===================== CONVERSIÓN TICKS -> RPM PARA CHASSIS =====================
    // REV HD Hex + UltraPlanetary nominal 15:1
    private static final double DRIVE_TICKS_PER_REV = 423.2;
    private static final double DRIVE_TICKS_TO_RPM = 60.0 / DRIVE_TICKS_PER_REV;

    @Override
    public void runOpMode() {

        // MOTORES DRIVE
        MotorDelDer = hardwareMap.get(DcMotorEx.class, "MotorDelDer");
        MotorTraDer = hardwareMap.get(DcMotorEx.class, "MotorTraDer");
        MotorDelIzq= hardwareMap.get(DcMotorEx.class, "MotorDelIzq");
        MotorTraIzq = hardwareMap.get(DcMotorEx.class, "MotorTraIzq");

        // SERVOS TILT
        TiltDer = hardwareMap.get(Servo.class, "TiltDer");
        TiltIzq = hardwareMap.get(Servo.class, "TiltIzq");

        // SERVO TURRET LEFT
        try {
            TurretLeftServo = hardwareMap.get(Servo.class, "TurretLeftServo");
            turretLeftServoPosition = TURRET_SERVO_LEFT_INIT;
            TurretLeftServo.setPosition(turretLeftServoPosition);
            telemetry.addLine("TurretLeftServo inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: TurretLeftServo no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // SERVO TURRET RIGHT
        try {
            TurretRightServo = hardwareMap.get(Servo.class, "TurretRightServo");
            turretRightServoPosition = TURRET_SERVO_RIGHT_INIT;
            TurretRightServo.setPosition(turretRightServoPosition);
            telemetry.addLine("TurretRightServo inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: TurretRightServo no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // SERVO GATE
        try {
            Gate = hardwareMap.get(Servo.class, "Gate");
            Gate.setPosition(GATE_CLOSED_POS); // Posición inicial cerrada
            telemetry.addLine("Gate inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Gate no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // MOTOR INTAKE
        try {
            Intake = hardwareMap.get(DcMotor.class, "Intake");
            Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Intake.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addLine("Intake inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Intake no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // MOTOR TURRET - PARA SEGUIMIENTO APRILTAG
        try {
            Turret = hardwareMap.get(DcMotorEx.class, "Turret");
            Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Turret inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Turret no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // MOTOR SHOOTER IZQUIERDO
        try {
            ShooterIzq = hardwareMap.get(DcMotorEx.class, "ShooterIzq");
            ShooterIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ShooterIzq.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("ShooterIzq inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: ShooterIzq no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // MOTOR SHOOTER DERECHO
        try {
            ShooterDer = hardwareMap.get(DcMotorEx.class, "ShooterDer");
            ShooterDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ShooterDer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("ShooterDer inicializado correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: ShooterDer no encontrado - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // SENSOR DE COLOR INTAKE
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "IntakeSensor");

        // LIMELIGHT 3A - APRILTAG TRACKING
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Pipeline 0 para AprilTag
            limelight.start();
            telemetry.addLine("Limelight 3A inicializada correctamente");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Limelight no encontrada - verificar nombre en configuración");
            telemetry.addData("Exception", e.getMessage());
        }

        // CONFIG DRIVE
        MotorDelDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorTraDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDelIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorTraIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorDelIzq.setDirection(DcMotor.Direction.FORWARD);
        MotorTraIzq.setDirection(DcMotor.Direction.FORWARD);
        MotorDelDer.setDirection(DcMotor.Direction.FORWARD);
        MotorTraDer.setDirection(DcMotor.Direction.REVERSE);

        // =========================== INIT POSITIONS ===========================
        // Esto mueve los servos a su posición inicial independiente al presionar "INIT"
        TiltDer.setPosition(TILT_DER_INITIAL_POS);
        TiltIzq.setPosition(TILT_IZQ_INITIAL_POS);

        telemetry.addLine("ORION2026 DRIVE & TILT LISTO");
        telemetry.addLine("Servos movidos a posición inicial.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================== DRIVE (Gamepad 2) ===========================
            double forward = -gamepad2.left_stick_y;
            double strafe  =  gamepad2.left_stick_x;
            double turn    =  gamepad2.right_stick_x;

            if (Math.abs(forward) < DEADBAND) forward = 0;
            if (Math.abs(strafe)  < DEADBAND) strafe  = 0;
            if (Math.abs(turn)    < DEADBAND) turn    = 0;

            double pDI = forward + strafe + turn;
            double pTI = forward - strafe + turn;
            double pDD = forward - strafe - turn;
            double pTD = forward + strafe - turn;

            double max = Math.max(Math.max(Math.abs(pDI), Math.abs(pTI)),
                    Math.max(Math.abs(pDD), Math.abs(pTD)));
            if (max > 1.0) {
                pDI /= max;
                pTI /= max;
                pDD /= max;
                pTD /= max;
            }

            MotorDelIzq.setPower(pDI);
            MotorTraIzq.setPower(pTI);
            MotorDelDer.setPower(pDD);
            MotorTraDer.setPower(pTD);

            // =========================== INTAKE ===========================
            // Gamepad 2: Left bumper = intake ON (con limite de 3 pelotas), Left trigger = intake ON (sin limite)
            //            Right bumper = intake REVERSE
            if (gamepad2.left_bumper && pelotasContadas < 3) {
                Intake.setPower(1.0);
            } else if (gamepad2.left_trigger > 0.5) {
                Intake.setPower(1.0);
            } else if (gamepad2.right_bumper) {
                Intake.setPower(-1.0);
            } else {
                Intake.setPower(0);
            }

            // =========================== CONTEO DE PELOTAS ===========================
            NormalizedRGBA colors = intakeSensor.getNormalizedColors();
            double intensidadLuz = colors.red + colors.green + colors.blue;
            boolean pelotaDetectada = intensidadLuz < UMBRAL_DETECCION;

            if (pelotaDetectada && !pelotaDetectadaAnteriormente) {
                pelotasContadas++;
            }
            pelotaDetectadaAnteriormente = pelotaDetectada;

            // =========================== TILT SERVOS (Gamepad 2) ===========================
            // Usamos el botón 'A' del gamepad 2 como un toggle
            boolean tiltButton = gamepad2.a;
            if (tiltButton && !tiltButtonPrev) {
                tiltActive = !tiltActive;
            }
            tiltButtonPrev = tiltButton;

            // Aplica las posiciones independientes
            if (tiltActive) {
                TiltDer.setPosition(TILT_DER_FINAL_POS);
                TiltIzq.setPosition(TILT_IZQ_FINAL_POS);
            } else {
                TiltDer.setPosition(TILT_DER_INITIAL_POS);
                TiltIzq.setPosition(TILT_IZQ_INITIAL_POS);
            }

            // =========================== APRILTAG TRACKING - LIMELIGHT ===========================
            // Toggle para alineación automática con botón 'B' del gamepad 1
            boolean aprilTagAlignButton = gamepad1.b;
            if (aprilTagAlignButton && !aprilTagAlignButtonPrev) {
                aprilTagAutoAlign = !aprilTagAutoAlign;
            }
            aprilTagAlignButtonPrev = aprilTagAlignButton;

            // Variables para datos del AprilTag
            String aprilTagData = "No detectado";
            double aprilTagTx = 0, aprilTagTy = 0;
            boolean aprilTagDetected = false;
            double turretPower = 0;

            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    // Obtener resultados de Fiducial (AprilTags)
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    if (!fiducialResults.isEmpty()) {
                        aprilTagDetected = true;
                        LLResultTypes.FiducialResult aprilTag = fiducialResults.get(0); // Primer AprilTag detectado
                        aprilTagData = "ID:" + aprilTag.getFiducialId() + " " + aprilTag.getFamily();
                        aprilTagTx = aprilTag.getTargetXDegrees(); // Ángulo horizontal hacia el AprilTag
                        aprilTagTy = aprilTag.getTargetYDegrees(); // Ángulo vertical hacia el AprilTag

                        // =========================== TURRET TRACKING ===========================
                        // Si el tracking está activado, el turret sigue al AprilTag
                        if (aprilTagAutoAlign && Turret != null) {
                            // Control PD para centrar el AprilTag (tx debe ir a 0)
                            double error = aprilTagTx;

                            // Calcular términos PD (sin integral para evitar inestabilidad)
                            double proportional = error * TURRET_KP;
                            double derivative = (error - turretLastError) * TURRET_KD;
                            turretLastError = error;

                            // Calcular potencia del motor
                            double rawPower = proportional + derivative;

                            // Aplicar suavizado exponencial para movimiento más fluido
                            turretSmoothedPower = turretSmoothedPower * (1 - TURRET_SMOOTHING) + rawPower * TURRET_SMOOTHING;

                            // Limitar potencia
                            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretSmoothedPower));

                            Turret.setPower(turretPower);
                        } else {
                            // Si no está activo, resetear el PID y detener motor
                            turretLastError = 0;
                            turretSmoothedPower = 0;
                            if (Turret != null) {
                                Turret.setPower(0);
                            }
                        }
                    } else {
                        // No se detecta AprilTag - detener turret
                        turretLastError = 0;
                        turretSmoothedPower = 0;
                        if (Turret != null) {
                            Turret.setPower(0);
                        }
                    }
                } else {
                    // Resultado inválido - detener turret
                    turretLastError = 0;
                    turretSmoothedPower = 0;
                    if (Turret != null) {
                        Turret.setPower(0);
                    }
                }
            }

            // =========================== CONTROL MANUAL DEL TURRET (Gamepad 1) ===========================
            // Si el tracking automático NO está activo, permitir control manual con joystick derecho
            if (!aprilTagAutoAlign && Turret != null) {
                double turretInput = gamepad1.right_stick_x;

                // Aplicar deadband
                if (Math.abs(turretInput) < DEADBAND) {
                    turretInput = 0;
                }

                // Control proporcional con el joystick
                Turret.setPower(turretInput * 0.5
                ); // 0.5 = potencia máxima manual
            }

            // =========================== TURRET SERVOS (Gamepad 1) ===========================
            // D-pad Up/Down mueve AMBOS servos incrementalmente al unísono
            // Left: dirección normal | Right: dirección INVERTIDA (matemáticamente)
            if (TurretLeftServo != null || TurretRightServo != null) {
                if (gamepad1.dpad_up) {
                    // Arriba - Left aumenta, Right disminuye
                    turretLeftServoPosition = Math.min(TURRET_SERVO_LEFT_MAX, turretLeftServoPosition + TURRET_SERVO_STEP);
                    turretRightServoPosition = Math.max(TURRET_SERVO_RIGHT_MIN, turretRightServoPosition - TURRET_SERVO_STEP);
                    if (TurretLeftServo != null) {
                        TurretLeftServo.setPosition(turretLeftServoPosition);
                    }
                    if (TurretRightServo != null) {
                        TurretRightServo.setPosition(turretRightServoPosition);
                    }
                } else if (gamepad1.dpad_down) {
                    // Abajo - Left disminuye, Right aumenta
                    turretLeftServoPosition = Math.max(TURRET_SERVO_LEFT_MIN, turretLeftServoPosition - TURRET_SERVO_STEP);
                    turretRightServoPosition = Math.min(TURRET_SERVO_RIGHT_MAX, turretRightServoPosition + TURRET_SERVO_STEP);
                    if (TurretLeftServo != null) {
                        TurretLeftServo.setPosition(turretLeftServoPosition);
                    }
                    if (TurretRightServo != null) {
                        TurretRightServo.setPosition(turretRightServoPosition);
                    }
                }
            }

            // =========================== SHOOTER (Gamepad 1) ===========================
            // Right bumper activa ambos motores del Shooter y abre el Gate después de un delay
            if (ShooterIzq != null && ShooterDer != null) {
                if (gamepad1.right_bumper) {
                    // Activar shooter
                    ShooterIzq.setPower(1.0);
                    ShooterDer.setPower(-1.0);

                    // Control del delay del Gate
                    if (!shooterActive) {
                        // Primer frame que se presiona el botón - marcar tiempo
                        shooterActive = true;
                        shooterStartTime = getRuntime();
                    }

                    // Verificar si ha pasado el delay para abrir el Gate
                    if (Gate != null && (getRuntime() - shooterStartTime) >= GATE_DELAY_SECONDS) {
                        Gate.setPosition(GATE_OPEN_POS);
                    }
                } else {
                    // Detener shooter
                    ShooterIzq.setPower(0);
                    ShooterDer.setPower(0);

                    // Cerrar Gate inmediatamente
                    if (Gate != null) {
                        Gate.setPosition(GATE_CLOSED_POS);
                    }

                    // Resetear estado
                    shooterActive = false;
                }
            }

            // =========================== TELEMETRÍA ===========================
            double rpmDI = MotorDelIzq.getVelocity() * DRIVE_TICKS_TO_RPM;
            double rpmTI = MotorTraIzq.getVelocity() * DRIVE_TICKS_TO_RPM;
            double rpmDD = MotorDelDer.getVelocity() * DRIVE_TICKS_TO_RPM;
            double rpmTD = MotorTraDer.getVelocity() * DRIVE_TICKS_TO_RPM;

            telemetry.addLine("=== DRIVE RPM ===");
            telemetry.addData("DI / TI", "%.0f / %.0f", rpmDI, rpmTI);
            telemetry.addData("DD / TD", "%.0f / %.0f", rpmDD, rpmTD);

            telemetry.addLine("=== TILT SERVOS ===");
            telemetry.addData("Estado", tiltActive ? "FINAL" : "INICIAL");
            telemetry.addData("Posición Der", tiltActive ? TILT_DER_FINAL_POS : TILT_DER_INITIAL_POS);
            telemetry.addData("Posición Izq", tiltActive ? TILT_IZQ_FINAL_POS : TILT_IZQ_INITIAL_POS);

            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Left Bumper", gamepad2.left_bumper ? "PRESIONADO" : "SUELTO");
            telemetry.addData("Left Trigger", gamepad2.left_trigger > 0.5 ? "PRESIONADO" : "SUELTO");
            telemetry.addData("Right Bumper", gamepad2.right_bumper ? "PRESIONADO" : "SUELTO");
            telemetry.addData("Power", "%.2f", Intake.getPower());
            telemetry.addData("Limite Pelotas", pelotasContadas >= 3 ? "ALCANZADO" : (pelotasContadas + "/3"));

            telemetry.addLine("=== SENSOR INTAKE ===");
            telemetry.addData("Pelotas contadas", pelotasContadas);
            telemetry.addData("Intensidad luz", "%.3f", intensidadLuz);

            telemetry.addLine("=== LIMELIGHT APRILTAG ===");
            telemetry.addData("AprilTag Detectado", aprilTagDetected ? "SÍ" : "NO");
            telemetry.addData("AprilTag Data", aprilTagData.length() > 20 ? aprilTagData.substring(0, 20) + "..." : aprilTagData);
            telemetry.addData("AprilTag Tx/Ty", aprilTagDetected ? String.format("%.1f° / %.1f°", aprilTagTx, aprilTagTy) : "N/A");
            telemetry.addData("Auto Align", aprilTagAutoAlign ? "ACTIVO (B para toggle)" : "INACTIVO");

            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Tracking", aprilTagAutoAlign && aprilTagDetected ? "ACTIVO" : "MANUAL");
            telemetry.addData("Power", "%.3f", turretPower);
            telemetry.addData("Manual", !aprilTagAutoAlign ? "(Right Stick X)" : "BLOCKED");
            telemetry.addData("Turret Left", "%.2f [%.0f-%.0f]", turretLeftServoPosition, TURRET_SERVO_LEFT_MIN * 100, TURRET_SERVO_LEFT_MAX * 100);
            telemetry.addData("Turret Right", "%.2f [%.0f-%.0f] INV", turretRightServoPosition, TURRET_SERVO_RIGHT_MIN * 100, TURRET_SERVO_RIGHT_MAX * 100);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Estado", gamepad1.right_bumper ? "ACTIVO" : "DETENIDO");
            if (ShooterIzq != null && ShooterDer != null) {
                telemetry.addData("Izq / Der", "%.2f / %.2f", ShooterIzq.getPower(), ShooterDer.getPower());
            }
            if (Gate != null) {
                telemetry.addData("Gate", gamepad1.right_bumper ? "ABIERTO" : "CERRADO");
            }

            telemetry.update();
        }

        // LIMELIGHT CLEANUP
        if (limelight != null) {
            limelight.stop();
        }
    }
}