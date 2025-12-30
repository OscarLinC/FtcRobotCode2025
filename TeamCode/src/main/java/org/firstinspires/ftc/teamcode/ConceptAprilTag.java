package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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
public class ConceptAprilTag extends LinearOpMode {

    private DriveTrain drivetrain;

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotorEx throwingMotor;
    private DcMotorEx bruce;
    private DcMotorEx intakeMotor;
    private Servo spinning_pad_discrete;
    private Servo throwPitchAdjuster;
    private Servo transferServo;
    private static final double BRUCE_POWER = 0.4;  // tune later

    // ----- 3-SLOT SPINNING PAD CONTROL -----
    private int padIndex = 0;  // 0,1,2
    private final double[] PAD_POS = {0.04, 0.115, 0.188};
    private final double[] PAD_POS_READY = {0.08, 0.154, 0.1515};
    double currentPitch;
    double pitchStep = 0.02;
    double targetPitch;
    boolean toggleIntake = false;
    boolean toggleSpitOut = false;
    boolean toggleThrow = false;

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
            if (curGamepad1.a && !prevGamepad1.a) {
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

            // Throwing toggle
            if (curGamepad2.b && !prevGamepad2.b) toggleThrow = !toggleThrow;

            throwingMotor.setPower(toggleThrow ? 1 : 0);

            if (toggleThrow) {
                spinning_pad_discrete.setPosition(PAD_POS_READY[padIndex]);
                if(Math.abs(spinning_pad_discrete.getPosition() - PAD_POS_READY[padIndex]) < 0.0005){
                    transferServo.setPosition(0.2);
                }


            }
            else {
                transferServo.setPosition(0.55);
                if (Math.abs(transferServo.getPosition()-0.55) < 0.05){
                    spinning_pad_discrete.setPosition(PAD_POS[padIndex]);
                }
            }




            // Bruce manual control
            bruce.setPower(curGamepad2.right_stick_x * -1);
            if (curGamepad2.right_stick_x == 0) bruce.setPower(0);

            // -------- SPINNING PAD 3-SLOT CONTROL --------
            if (curGamepad2.dpad_right && !prevGamepad2.dpad_right) {
                padIndex = (padIndex + 1) % 3;
            }
            if (curGamepad2.dpad_left && !prevGamepad2.dpad_left) {
                padIndex = (padIndex + 2) % 3; // decrement with wrap
            }


            telemetry.addData("Pad Index", padIndex);
            telemetry.addData("Pad Angle (deg)", padIndex * 120);
            telemetry.addData("Pad Position", "%.3f", PAD_POS[padIndex]);

            // -------- APRILTAG AUTO TU           0N --------
            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                double error = (tag.center.x - 320) / 320.0;
                error = Math.max(-1, Math.min(1, error));
                double distance = tag.ftcPose.range;
                targetPitch = (distance - 100) / 500;
                drivetrain.drive(gamepad1, 1);
                bruce.setPower(0.75 * error);

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
