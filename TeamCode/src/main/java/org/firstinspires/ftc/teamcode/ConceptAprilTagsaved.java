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

@TeleOp(name = "Concept: AprilTag", group = "Concept")
public class ConceptAprilTagsaved extends LinearOpMode {

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
    private final double[] PAD_POS = {0.04, 0.115, 0.188};
    private final double[] PAD_POS_READY = {0.085, 0.159, 0.1565};
    double currentPitch;
    double pitchStep = 0.02;
    double targetPitch;
    boolean toggleIntake = false;
    boolean toggleSpitOut = false;
    boolean toggleThrow1 = false;
    private int sequenceIndex = 0;


    boolean toggleThrow = false;
    private int motifAprilTag;

    // ID 21 = GPP
    // ID 22 = PGP
    // ID 23 = PPG
    boolean toggleThrow2 = false;
    private int[] transferServoSequence = {1, 2, 3};
    @Override
    public void runOpMode() {

        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        throwingMotor = hardwareMap.get(DcMotorEx.class, "Throwing Motor");
        bruce = hardwareMap.get(DcMotorEx.class, "Bruce");
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


///remember to change this
//            // Throwing toggle
//            if (curGamepad2.b && !prevGamepad2.b) toggleThrow = !toggleThrow;
//
//            throwingMotor.setPower(toggleThrow ? 1 : 0);
//
//            if (toggleThrow) {
//                spinning_pad_discrete.setPosition(PAD_POS_READY[padIndex]);
//                if(Math.abs(spinning_pad_discrete.getPosition() - PAD_POS_READY[padIndex]) < 0.0005){
//                    transferServo.setPosition(0.2);
//                }
//
//
//            }
//            else {
//                transferServo.setPosition(0.55);
//                if (Math.abs(transferServo.getPosition()-0.55) < 0.05){
//                    spinning_pad_discrete.setPosition(PAD_POS[padIndex]);
//                }
//            }


            // 1. Add this variable at the top with your other private variables


// ... inside the while(opModeIsActive()) loop ...

// 1. Ensure these are defined at the top of your class (outside runOpMode)

// ... inside the while(opModeIsActive()) loop ...
            // -------- SPINNING PAD 3-SLOT CONTROL --------
            // 1. INPUT HANDLING: D-pad for choosing slots
// 1. INPUT HANDLING: Manual D-pad (still works to skip slots)

            //Right = purple, left = green
//            if (curGamepad2.dpad_right && !prevGamepad2.dpad_right) {
//                storedColors[padIndex] = 1;
//                padIndex = (padIndex + 1) % 3;
//
//            }
//            if (curGamepad2.dpad_left && !prevGamepad2.dpad_left) {
//                storedColors[padIndex] = 0;
//                padIndex = (padIndex + 1) % 3;
//            }

            // Color sorting
            NormalizedRGBA colors = colorsensor.getNormalizedColors();
            float normRed, normBlue, normGreen;
            normRed = colors.red / colors.alpha;
            normGreen = colors.green / colors.alpha;
            normBlue = colors.blue / colors.alpha;
            //purple: 0.0069, 0.008, 0.0124
            //green:  0.0034, 0.0116, 0.0087
            if (0.003 < normRed && normRed < 0.004 && normGreen> 0.01 && normGreen < 0.014 && normBlue > 0.008 && normBlue < 0.011 && colorSensorTimer.seconds() > 0.5 && (storedColors[0] == 2 || storedColors[1] == 2 ||storedColors[2] == 2)) {
                colorSensorTimer.reset();
                storedColors[padIndex] = 1;
                padIndex = (padIndex + 1) % 3;

            }
            if (0.005 < normRed && normRed < 0.008 && normGreen> 0.006 && normGreen < 0.009 && normBlue > 0.01 && normBlue < 0.014 && colorSensorTimer.seconds() > 0.5 && (storedColors[0] == 2 || storedColors[1] == 2 ||storedColors[2] == 2)) {
                colorSensorTimer.reset();
                storedColors[padIndex] = 0;
                padIndex = (padIndex + 1) % 3;
            }

            switch(motifAprilTag){
                case 1:
                    //GPP
                    //0 = green, 1=purple, 2=none
                    for (int storedColorIndex=0; storedColorIndex<2; storedColorIndex++){
                        if (storedColors[storedColorIndex] == 0){//if green go first
                            transferServoSequence[storedColorIndex] = 1;
                        }

                        else if (storedColors[storedColorIndex] == 2){//if none go last
                            transferServoSequence[storedColorIndex] = 3;
                        }

                        else if (storedColors[storedColorIndex] == 1 && transferServoSequence[storedColorIndex] != 3){
                            transferServoSequence[storedColorIndex] = 2;
                        }
                    }
                case 2:
                    //PGP
                    for (int storedColorIndex=0; storedColorIndex<2; storedColorIndex++){
                        if (storedColors[storedColorIndex] == 0){//if green go second
                            transferServoSequence[storedColorIndex] = 2;
                        }

                        else if (storedColors[storedColorIndex] == 2){//if none go last
                            transferServoSequence[storedColorIndex] = 3;
                        }

                        else if (storedColors[storedColorIndex] == 1 && transferServoSequence[storedColorIndex] != 3){
                            transferServoSequence[storedColorIndex] = 1; //if purple and got purple first, go first
                        }
                    }
                case 3:
                    //PPG
                    for (int storedColorIndex=0; storedColorIndex<2; storedColorIndex++){
                        if (storedColors[storedColorIndex] == 0){//if green go last
                            transferServoSequence[storedColorIndex] = 3;
                        }

                        else if (storedColors[storedColorIndex] == 2){//if none go second
                            transferServoSequence[storedColorIndex] = 2;
                        }

                        else if (storedColors[storedColorIndex] == 1 && transferServoSequence[storedColorIndex] != 2){
                            transferServoSequence[storedColorIndex] = 1; //if purple and got purple first, go first
                        }
                    }

            }

// 2. INPUT HANDLING: B Button
            if (curGamepad2.b && !prevGamepad2.b) {
                toggleThrow = !toggleThrow;
                if (toggleThrow) {
                    sequenceStep = 1;
                    sequenceTimer.reset();
                } else {
                    sequenceStep = 3;
                    sequenceTimer.reset();
                }
            }

// 3. MOTOR POWER
            throwingMotor.setPower(toggleThrow ? 1 : 0);

// 4. THE UPDATED STATE MACHINE
            switch (sequenceStep) {

                case 0: // IDLE
                    spinning_pad_discrete.setPosition(PAD_POS[padIndex]);
                    transferServo.setPosition(0.55);
                    break;

                case 1: // Move pad to shooting alignment

                    sequenceIndex = 0;
                    spinning_pad_discrete.setPosition(PAD_POS_READY[0]);
                    if (sequenceTimer.seconds() > 0.3) {
                        sequenceStep = transferServoSequence[sequenceIndex] + 1;
                        sequenceTimer.reset();
                    }
                    break;

                case 2: // Raise 1st arm to shoot
                    sequenceTimer.reset();
                    transferServo.setPosition(0.2);
                    sequenceIndex += 1;
                    if (sequenceTimer.seconds() > 0.3) {
                        transferServo.setPosition(0.55);
                        if (sequenceIndex <=1){
                            sequenceStep = transferServoSequence[sequenceIndex] + 1;
                        }
                        else {sequenceIndex = 0; sequenceStep=0;
                            storedColors[0] = 2;
                            storedColors[1] = 2;
                            storedColors[2] = 2;
                        }

                    }
                    break;

                case 3: // Raise 2nd arm to shoot
                    sequenceTimer.reset();
                    transferServo2.setPosition(0.2);
                    sequenceIndex += 1;
                    if (sequenceTimer.seconds() > 0.3) {
                        transferServo2.setPosition(0.55);
                        if (sequenceIndex <=1){
                            sequenceStep = transferServoSequence[sequenceIndex] + 1;
                        }
                        else {sequenceIndex = 0; sequenceStep=0;
                            storedColors[0] = 2;
                            storedColors[1] = 2;
                            storedColors[2] = 2;
                        }
                    }
                    break;


                case 4: // Raise 2nd arm to shoot
                    sequenceTimer.reset();
                    transferServo3.setPosition(0.2);
                    sequenceIndex += 1;
                    if (sequenceTimer.seconds() > 0.3) {
                        transferServo3.setPosition(0.55);
                        if (sequenceIndex <=1){
                            sequenceStep = transferServoSequence[sequenceIndex] + 1;
                        }
                        else {sequenceIndex = 0; sequenceStep=0;
                            storedColors[0] = 2;
                            storedColors[1] = 2;
                            storedColors[2] = 2;
                        }
                    }
                    break;

//                case 4: // Move pad to the NEW position
//                    spinning_pad_discrete.setPosition(PAD_POS[padIndex]);
//                    if (sequenceTimer.seconds() > 0.2) {
//                        sequenceStep = 0; // Return to IDLE at the next ball
//                    }
//                    break;
            }



            // Bruce manual control
            bruce.setPower(curGamepad2.right_stick_x * -1);


            if (curGamepad2.right_stick_x == 0) bruce.setPower(0);


            telemetry.addData("red", normRed);
            telemetry.addData("green", normGreen);
            telemetry.addData("blue", normBlue);


            telemetry.addData("Pad Index", padIndex);
            telemetry.addData("Pad Angle (deg)", padIndex * 120);
            telemetry.addData("Pad Position", "%.3f", PAD_POS[padIndex]);

            // -------- APRILTAG AUTO TU           0N --------
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

            // -------- TELEMETRY --------
            telemetryAprilTag();
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();



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

        for (AprilTagDetection d : detections) {
            if (d.metadata != null) {
                telemetry.addLine(String.format(
                        "ID %d | Range %.1f cm | Bearing %.1f deg | Yaw %.1f deg",
                        d.id,
                        d.ftcPose.range,
                        d.ftcPose.bearing,
                        d.ftcPose.yaw
                ));
            } else {
                telemetry.addLine(String.format("ID %d | Unknown", d.id));
            }
        }
    }
}
