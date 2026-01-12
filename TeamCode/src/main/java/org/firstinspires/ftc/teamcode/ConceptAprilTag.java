package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

@TeleOp(name = "Concept: AprilTag Final 20250109", group = "Concept")
public class ConceptAprilTag extends LinearOpMode {

    private DriveTrain drivetrain;

    private static final boolean USE_WEBCAM = true;
    private NormalizedColorSensor colorsensor;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private com.qualcomm.robotcore.util.ElapsedTime sequenceTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private com.qualcomm.robotcore.util.ElapsedTime colorSensorTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private int sequenceStep = 0; // 0 = idle, 1 = move pad, 2 = raise servo, 3 = lower servo, 4 = reset pad

    private DcMotorEx throwingMotor;
    private DcMotorEx bruce;
    private DcMotorEx intakeMotor;
    private Servo spinning_pad_discrete;
    private Servo throwPitchAdjuster;
    private Servo transferServo;

    private Servo transferServo2;
    private Servo transferServo3;

    private int debugColor;
    private int[] storedColors = {2, 2, 2}; //0 = green, 1=purple, 2=none
    private static final double BRUCE_POWER = 0.4;  // tune later

    // ----- 3-SLOT SPINNING PAD CONTROL -----
    private int padIndex = 0;  // 0,1,2
    private final double[] PAD_POS = {0.027, 0.46, 0.91};
    private final double[] PAD_POS_READY = {0.245, 0, 0};
    double currentPitch;
    double pitchStep = 0.02;
    double targetPitch;
    boolean toggleIntake = false;
    boolean toggleSpitOut = false;
    boolean toggleThrow1 = false;
    private int sequenceIndex = 0;


    boolean toggleThrow = false;
    private int motifAprilTag;

    // Toggle for Y button
    private boolean useMotifToggle = false;

    // ID 21 = GPP
    // ID 22 = PGP
    // ID 23 = PPG
    boolean toggleThrow2 = false;
    private int[] transferServoSequence = {1, 2, 3};
    // ----- BRUCE (HOOD) LIMITS -----
    // TODO: CHANGE THIS TO MATCH YOUR MOTOR (e.g., 537.7 for GoBilda 312RPM, 28 for Rev Core Hex)
    final double BRUCE_TICKS_PER_REV = 537.7;
    // 45 degrees = 1/8th of a circle
    final int BRUCE_LIMIT_TICKS = (int)(BRUCE_TICKS_PER_REV * (45.0 / 360.0));
    @Override
    public void runOpMode() {

        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        throwingMotor = hardwareMap.get(DcMotorEx.class, "Throwing Motor");
        bruce = hardwareMap.get(DcMotorEx.class, "Bruce");
        bruce.setDirection(DcMotorSimple.Direction.REVERSE);
        // --- NEW CODE: CONFIGURE BRUCE ENCODER ---
        // Reset the encoder so the current position becomes "0"
        bruce.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Turn motor back on in manual power mode (but it will now track position)
        bruce.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Optional: Turn on braking so it holds position better
        bruce.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake Motor");
        spinning_pad_discrete = hardwareMap.get(Servo.class, "Spinning Pad");
        throwPitchAdjuster = hardwareMap.get(Servo.class, "Throw Pitch Adjuster");
        transferServo = hardwareMap.get(Servo.class, "Transfer Servo");
        transferServo2=hardwareMap.get(Servo.class, "Daniel Chen Transfer Servo 2");
        transferServo3=hardwareMap.get(Servo.class, "Oscar Lin Transfer Servo 3");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "Color Sensor");
        bruce.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new DriveTrain(
                hardwareMap.get(DcMotorEx.class, "M1"),
                hardwareMap.get(DcMotorEx.class, "M2"),
                hardwareMap.get(DcMotorEx.class, "M3"),
                hardwareMap.get(DcMotorEx.class, "M4")
        );

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

            // Pitch manual control
            if (curGamepad2.dpad_up) throwPitchAdjuster.setPosition(0.6);
            else if (curGamepad2.dpad_down) throwPitchAdjuster.setPosition(0.11);

            // Intake toggle
            if ((storedColors[0] != 2 && storedColors[1] != 2 && storedColors[2] != 2)){
                toggleIntake = false;
            }
            else if (curGamepad1.a && !prevGamepad1.a) {
                toggleIntake = !toggleIntake;
                toggleSpitOut = false;
            }


            if (curGamepad1.x && !prevGamepad1.x) {
                toggleSpitOut = !toggleSpitOut;
                toggleIntake = false;
            }

            if (toggleIntake) intakeMotor.setPower(1);
            else if (toggleSpitOut) intakeMotor.setPower(-1);
            else intakeMotor.setPower(0);

            // Color sorting - Always active to catch "random" balls
            NormalizedRGBA colors = colorsensor.getNormalizedColors();
            float normRed = colors.red / colors.alpha;
            float normGreen = colors.green / colors.alpha;
            float normBlue = colors.blue / colors.alpha;

            if (colorSensorTimer.seconds() > 0.5 && (storedColors[0] == 2 || storedColors[1] == 2 || storedColors[2] == 2)) {
                // Purple detection
                if (0.003 < normRed && normRed < 0.004 && normGreen > 0.01 && normGreen < 0.014 && normBlue > 0.008 && normBlue < 0.011) {
                    colorSensorTimer.reset();
                    storedColors[padIndex] = 1;
                    padIndex = (padIndex + 1) % 3;
                }
                // Green detection
                else if (0.005 < normRed && normRed < 0.008 && normGreen > 0.006 && normGreen < 0.009 && normBlue > 0.01 && normBlue < 0.014) {
                    colorSensorTimer.reset();
                    storedColors[padIndex] = 0;
                    padIndex = (padIndex + 1) % 3;
                }
            }

            // Y Button Toggle logic
            if (curGamepad1.y && !prevGamepad1.y) {
                useMotifToggle = !useMotifToggle;
            }

            // Motif Logic Calculation
            if (useMotifToggle) {
                switch(motifAprilTag){
                    case 1: // GPP
                        for (int i=0; i<3; i++){
                            if (storedColors[i] == 0) transferServoSequence[i] = 1;
                            else if (storedColors[i] == 2) transferServoSequence[i] = 3;
                            else transferServoSequence[i] = 2;
                        }
                        break;
                    case 2: // PGP
                        for (int i=0; i<3; i++){
                            if (storedColors[i] == 0) transferServoSequence[i] = 2;
                            else if (storedColors[i] == 2) transferServoSequence[i] = 3;
                            else transferServoSequence[i] = 1;
                        }
                        break;
                    case 3: // PPG
                        for (int i=0; i<3; i++){
                            if (storedColors[i] == 0) transferServoSequence[i] = 3;
                            else if (storedColors[i] == 2) transferServoSequence[i] = 2;
                            else transferServoSequence[i] = 1;
                        }
                        break;
                    default:
                        transferServoSequence[0] = 1; transferServoSequence[1] = 2; transferServoSequence[2] = 3;
                        break;
                }
            } else {
                // Default sequence (Manual Mode)
                transferServoSequence[0] = 1;
                transferServoSequence[1] = 2;
                transferServoSequence[2] = 3;
            }

            // 2. INPUT HANDLING: B Button
            if (curGamepad2.b && !prevGamepad2.b) {
                toggleThrow = !toggleThrow;
                if (toggleThrow) {
                    sequenceStep = 1;
                    sequenceTimer.reset();
                } else {
                    sequenceStep = 0;
                }
            }

            // 3. MOTOR POWER
            throwingMotor.setPower(toggleThrow ? 1 : 0);

            // 4. THE UPDATED STAT[E MACHINE
            // 4. THE UPDATED STATE MACHINE
            switch (sequenceStep) {

                case 0: // IDLE
                    spinning_pad_discrete.setPosition(PAD_POS[padIndex]);
                    transferServo.setPosition(0.55);
                    transferServo2.setPosition(0.25);
                    transferServo3.setPosition(0.55);
                    break;

                case 1: // Move pad to shooting alignment
                    sequenceIndex = 0;
                    spinning_pad_discrete.setPosition(0.26);
                    sleep(1500);
                    if (sequenceTimer.seconds() > 0.3) {
                        // Automatically go to the first ball step
                        sequenceStep = transferServoSequence[sequenceIndex] + 1;
                        sequenceTimer.reset();
                    }
                    break;

                case 2: // Raise 1st arm to shoot (transferServo)
                    transferServo.setPosition(0.1); // Move to "Ready" position

                    // Wait 0.5s, then automatically fire
                    if (sequenceTimer.seconds() > 0.5) {
                        sleep(1500);
                        spinning_pad_discrete.setPosition(0.245);
                        sleep(1500);
                        transferServo.setPosition(0.75); // Fire/Return
                        sleep(1500); // FREEZES ROBOT
                        advanceSequence();
                    }
                    break;

                case 3: // Raise 2nd arm to shoot (transferServo2)
                    sleep(1500);
                    transferServo2.setPosition(0.65); // Move to "Ready" position
                    sleep(1500);

                    // Wait 0.5s, then automatically fire
                    if (sequenceTimer.seconds() > 0.5) {
                        sleep(1500);
                        transferServo2.setPosition(0.1); // Fire/Return

                        sleep(300); // FREEZES ROBOT
                        advanceSequence();
                    }
                    break;

                case 4: // Raise 3rd arm to shoot (transferServo3)
                    // Note: No initial position set here in your snippet, preserving your logic

                    // Wait 0.5s, then automatically fire
                    if (sequenceTimer.seconds() > 0.5) {
                        spinning_pad_discrete.setPosition(0.235);
                        sleep(2000); // FREEZES ROBOT
                        transferServo3.setPosition(0.9


                        );
                        sleep(2000); // FREEZES ROBOT (4 Seconds!)
                        transferServo3.setPosition(0.55);
                        sleep(300);  // FREEZES ROBOT
                        advanceSequence();
                    }
                    break;
            }

            // Bruce manual control
//            bruce.setPower(curGamepad2.right_stick_x * -1);
//            if (curGamepad2.right_stick_x == 0) bruce.setPower(0);


            // -------- BRUCE (HOOD) MANUAL CONTROL WITH LIMITS --------
            double joystickInput = curGamepad2.right_stick_x * -1;
            int currentPos = bruce.getCurrentPosition();

            // Logic: If trying to go POSITIVE (Left?) but already past limit -> Stop
            //        If trying to go NEGATIVE (Right?) but already past negative limit -> Stop

            if (joystickInput > 0 && currentPos > BRUCE_LIMIT_TICKS) {
                bruce.setPower(0); // Hit the positive (left) barrier
            }
            else if (joystickInput < 0 && currentPos < -BRUCE_LIMIT_TICKS) {
                bruce.setPower(0); // Hit the negative (right) barrier
            }
            else {
                // Within safe range, allow movement
                bruce.setPower(joystickInput);
            }

            // Telemetry to help you debug limits
            telemetry.addData("Bruce Pos", currentPos);
            telemetry.addData("Bruce Limit", "+/- " + BRUCE_LIMIT_TICKS);


            // -------- APRILTAG AUTO --------
            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                if (tag.id == 24 || tag.id == 20){
                    double error = (tag.center.x - 320) / 320.0;
                    error = Math.max(-1, Math.min(1, error));
                    double distance = tag.ftcPose.range;
                    targetPitch = (distance - 100) / 500;
                    drivetrain.drive(gamepad1, 1);
                    bruce.setPower(0.75 * error);
                }
                else {
                    motifAprilTag = tag.id - 20;
                }
            } else {
                drivetrain.drive(gamepad1, 1);
                currentPitch = throwPitchAdjuster.getPosition();
                targetPitch = currentPitch - curGamepad2.right_stick_y * pitchStep;
            }

            if (targetPitch >= 0.6) targetPitch = 0.6;
            else if (targetPitch <= 0.11) targetPitch = 0.11;
            throwPitchAdjuster.setPosition(targetPitch);

            telemetryAprilTag();
            telemetry.update();
            sleep(20);
        }
        visionPortal.close();
    }

    // Helper method to handle moving to the next ball in the motif
    private void advanceSequence() {
        sequenceIndex++;
        if (sequenceIndex < 3) {
            sequenceStep = transferServoSequence[sequenceIndex] + 1;
            sequenceTimer.reset();
        } else {
            // End of motif: Reset everything
            sequenceStep = 0;
            sequenceIndex = 0;
            toggleThrow = false;
            storedColors[0] = 2;
            storedColors[1] = 2;
            storedColors[2] = 2;
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(2);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal.resumeStreaming();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        telemetry.addData("Motif ID", motifAprilTag);
        telemetry.addData("Motif Sorting", useMotifToggle ? "ON" : "OFF (Manual 1-2-3)");
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) {
                telemetry.addLine(String.format("ID %d | Range %.1f cm | Bearing %.1f deg", d.id, d.ftcPose.range, d.ftcPose.bearing));
            }
        }
    }
}