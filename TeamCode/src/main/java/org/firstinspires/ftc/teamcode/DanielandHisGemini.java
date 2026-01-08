package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Concept: AprilTag Fixed", group = "Concept")
public class DanielandHisGemini extends LinearOpMode {

    private DriveTrain drivetrain;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private NormalizedColorSensor colorsensor;

    private com.qualcomm.robotcore.util.ElapsedTime sequenceTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private com.qualcomm.robotcore.util.ElapsedTime colorSensorTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    private int sequenceStep = 0;
    private int sequenceIndex = 0;
    private int motifAprilTag = 0;
    private int padIndex = 0;
    private int[] storedColors = {2, 2, 2}; // 0=green, 1=purple, 2=none
    private int[] transferServoSequence = {1, 2, 3};

    private DcMotorEx throwingMotor, bruce, intakeMotor;
    private Servo spinning_pad_discrete, throwPitchAdjuster, transferServo, transferServo2, transferServo3;

    private final double[] PAD_POS = {0.04, 0.115, 0.188};
    private final double[] PAD_POS_READY = {0.085, 0.159, 0.1565};
    private final double SERVO_HOME = 0.55;
    private final double SERVO_HOME2 = 0.1;
    private final double SERVO_HOME3 = 0.55;
    private final double SERVO_FIRE = 0.2;
    private final double SERVO_FIRE2 = 0.55;

    private final double SERVO_FIRE3 = 0.2;

    boolean toggleIntake = false;
    boolean toggleSpitOut = false;
    boolean toggleThrow = false;
    double targetPitch = 0.11;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        // Ensure everything is at HOME during Init
        transferServo.setPosition(SERVO_HOME);
        transferServo2.setPosition(SERVO_HOME);
        transferServo3.setPosition(SERVO_HOME);
        spinning_pad_discrete.setPosition(PAD_POS[0]);

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad curGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();
        Gamepad curGamepad2 = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            prevGamepad1.copy(curGamepad1);
            curGamepad1.copy(gamepad1);
            prevGamepad2.copy(curGamepad2);
            curGamepad2.copy(gamepad2);

            // --- 1. MOTIF & SEQUENCE LOGIC ---
            updateMotifSequence();

            // --- 2. INTAKE & COLOR SENSING ---
            handleIntake(curGamepad1, prevGamepad1);
            handleColorSensor();

            // --- 3. STATE MACHINE CALCULATION ---
            handleShootingSequence(curGamepad2, prevGamepad2);

            // --- 4. SERVO POSITION ARBITRATION ---
            // We use variables so the State Machine and Manual Buttons don't fight
            double t1 = SERVO_HOME;
            double t2 = SERVO_HOME2;
            double t3 = SERVO_HOME3;

            // Apply positions based on the current active step in the sequence
            if (sequenceStep == 2) t1 = SERVO_FIRE;
            if (sequenceStep == 3) t2 = SERVO_FIRE2;
            if (sequenceStep == 4) t3 = SERVO_FIRE3;

            // MANUAL OVERRIDES (RB = Servo 1, LB = Servo 2)
            if (curGamepad2.right_bumper) t1 = 0.7; // Your specific test position
            if (curGamepad2.left_bumper)  t2 = 0.7;

            // THE ONLY HARDWARE CALLS
            transferServo.setPosition(t1);
            transferServo2.setPosition(t2);
            transferServo3.setPosition(t3);

            // --- 5. DRIVETRAIN & BRUCE ---
            handleDrivetrain(curGamepad2);

            telemetry.addData("Step", sequenceStep);
            telemetry.addData("Motif ID", motifAprilTag);
            telemetry.addData("Pad Index", padIndex);
            telemetry.update();
            sleep(20);
        }
        visionPortal.close();
    }

    private void handleShootingSequence(Gamepad cur, Gamepad prev) {
        if (cur.b && !prev.b) {
            toggleThrow = !toggleThrow;
            if (toggleThrow) {
                sequenceStep = 1;
                sequenceTimer.reset();
            } else {
                sequenceStep = 0; // Cancel
            }
        }

        throwingMotor.setPower(toggleThrow ? 1 : 0);

        switch (sequenceStep) {
            case 0: // IDLE
                spinning_pad_discrete.setPosition(PAD_POS[padIndex]);
                break;

            case 1: // ALIGN PAD
                sequenceIndex = 0;
                spinning_pad_discrete.setPosition(PAD_POS_READY[0]);
                if (sequenceTimer.seconds() > 0.4) {
                    sequenceStep = transferServoSequence[sequenceIndex] + 1; // Go to 2, 3, or 4
                    sequenceTimer.reset();
                }
                break;

            case 2: // SERVO 1 ACTIVE
            case 3: // SERVO 2 ACTIVE
            case 4: // SERVO 3 ACTIVE
                if (sequenceTimer.seconds() > 0.5) { // Wait for arm to finish push
                    sequenceIndex++;
                    if (sequenceIndex < 3) {
                        sequenceStep = transferServoSequence[sequenceIndex] + 1;
                    } else {
                        // Sequence Finished
                        sequenceStep = 0;
                        toggleThrow = false;
                        storedColors[0] = 2; storedColors[1] = 2; storedColors[2] = 2;
                    }
                    sequenceTimer.reset();
                }
                break;
        }
    }

    private void updateMotifSequence() {
        // ID 21 = GPP, 22 = PGP, 23 = PPG
        for (int i = 0; i < 3; i++) {
            if (storedColors[i] == 2) { transferServoSequence[i] = 3; continue; } // Empty slots last

            if (motifAprilTag == 1) { // GPP
                if (storedColors[i] == 0) transferServoSequence[i] = 1; // Green first
                else transferServoSequence[i] = 2;
            }
            else if (motifAprilTag == 2) { // PGP
                if (storedColors[i] == 0) transferServoSequence[i] = 2; // Green second
                else transferServoSequence[i] = 1;
            }
            else { // Default or PPG
                if (storedColors[i] == 0) transferServoSequence[i] = 3; // Green last
                else transferServoSequence[i] = 1;
            }
        }
    }

    private void handleColorSensor() {
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        if (colorSensorTimer.seconds() > 0.6) {
            // Green Detect
            if (r < 0.005 && g > 0.01) {
                storedColors[padIndex] = 0;
                padIndex = (padIndex + 1) % 3;
                colorSensorTimer.reset();
            }
            // Purple Detect
            else if (r > 0.005 && b > 0.01) {
                storedColors[padIndex] = 1;
                padIndex = (padIndex + 1) % 3;
                colorSensorTimer.reset();
            }
        }
    }

    private void handleIntake(Gamepad cur, Gamepad prev) {
        if (cur.a && !prev.a) { toggleIntake = !toggleIntake; toggleSpitOut = false; }
        if (cur.x && !prev.x) { toggleSpitOut = !toggleSpitOut; toggleIntake = false; }

        if (toggleIntake) intakeMotor.setPower(1);
        else if (toggleSpitOut) intakeMotor.setPower(-1);
        else intakeMotor.setPower(0);
    }

    private void handleDrivetrain(Gamepad cur2) {
        if (!aprilTag.getDetections().isEmpty()) {
            AprilTagDetection tag = aprilTag.getDetections().get(0);
            if (tag.id == 24 || tag.id == 20) {
                double error = (tag.center.x - 320) / 320.0;
                bruce.setPower(0.75 * error);
                targetPitch = (tag.ftcPose.range - 100) / 500.0;
            } else {
                motifAprilTag = tag.id - 20;
            }
        } else {
            bruce.setPower(-cur2.right_stick_x);
            targetPitch -= cur2.right_stick_y * 0.02;
        }

        if (targetPitch > 0.6) targetPitch = 0.6;
        if (targetPitch < 0.11) targetPitch = 0.11;
        throwPitchAdjuster.setPosition(targetPitch);
        drivetrain.drive(gamepad1, 1);
    }

    private void initHardware() {
        throwingMotor = hardwareMap.get(DcMotorEx.class, "Throwing Motor");
        bruce = hardwareMap.get(DcMotorEx.class, "Bruce");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake Motor");
        spinning_pad_discrete = hardwareMap.get(Servo.class, "Spinning Pad");
        throwPitchAdjuster = hardwareMap.get(Servo.class, "Throw Pitch Adjuster");
        transferServo = hardwareMap.get(Servo.class, "Transfer Servo");
        transferServo2 = hardwareMap.get(Servo.class, "Daniel Chen Transfer Servo 2");
        transferServo3 = hardwareMap.get(Servo.class, "Oscar Lin Transfer Servo 3");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "Color Sensor");
        bruce.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain = new DriveTrain(
                hardwareMap.get(DcMotorEx.class, "M1"), hardwareMap.get(DcMotorEx.class, "M2"),
                hardwareMap.get(DcMotorEx.class, "M3"), hardwareMap.get(DcMotorEx.class, "M4")
        );
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}