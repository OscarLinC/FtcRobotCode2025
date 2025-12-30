package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AutoRed", group = "Autonomous")
public class AutoRed extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Bruce motor and servo
    private DcMotorEx bruce;
    private Servo spinning_pad_discrete;

    // AprilTag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Encoder constants
    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBILDA 5202/5203
    static final double WHEEL_DIAMETER_CM = 9.6; // 96mm wheel
    static final double COUNTS_PER_CM =
            COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_CM);

    // Servo variables
    int servoStep = 0;
    final int SERVO_STEPS = 15;
    double servo_position = 0;
    double step = 0;

    @Override
    public void runOpMode() {

        // Hardware mapping
        frontLeft  = hardwareMap.get(DcMotor.class, "M1");
        frontRight = hardwareMap.get(DcMotor.class, "M3");
        backLeft   = hardwareMap.get(DcMotor.class, "M2");
        backRight  = hardwareMap.get(DcMotor.class, "M4");

        bruce = hardwareMap.get(DcMotorEx.class, "Bruce");
        spinning_pad_discrete = hardwareMap.get(Servo.class, "Spinning Pad");

        // Motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        resetEncoders();

        // Initialize AprilTag
        initAprilTag();

        telemetry.addLine("Ready for Autonomous");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =====================
        // AUTONOMOUS SEQUENCE
        // =====================

        // Example: detect tag, turn to align
        if (!aprilTag.getDetections().isEmpty()) {
            AprilTagDetection tag = aprilTag.getDetections().get(0);
            double error = (tag.center.x - 320) / 320.0;
            error = Math.max(-1, Math.min(1, error));
            bruce.setPower(-error); // auto turn adjustment
            sleep(500);
            bruce.setPower(0);
        }

        // Drive forward 60 cm
        driveForward(60, 0.5);
        sleep(500);

        // Strafe right 30 cm
        strafeRight(30, 0.5);
        sleep(500);

        // Turn left ~90 degrees (rough)
        turnLeft(90, 0.4);
        sleep(500);

        // Move forward small distance 15 cm
        driveForward(15, 0.3);

        // Example: rotate servo 1 step
        rotateServoStep();

        telemetry.addLine("Autonomous Complete");
        telemetry.update();

        // Close Vision
        visionPortal.close();
    }

    // =====================
    // MOVEMENT METHODS
    // =====================

    private void driveForward(double cm, double power) {
        int moveCounts = (int) (cm * COUNTS_PER_CM);
        setTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);
        runToPosition(power);
    }

    private void strafeRight(double cm, double power) {
        int moveCounts = (int) (cm * COUNTS_PER_CM);
        setTargetPosition(moveCounts, -moveCounts, -moveCounts, moveCounts);
        runToPosition(power);
    }

    private void turnLeft(double degrees, double power) {
        double cm = degrees / 90.0 * 38.0; // rough estimate
        int moveCounts = (int) (cm * COUNTS_PER_CM);
        setTargetPosition(-moveCounts, moveCounts, -moveCounts, moveCounts);
        runToPosition(power);
    }

    // =====================
    // HELPER METHODS
    // =====================

    private void setTargetPosition(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(backRight.getCurrentPosition() + br);
    }

    private void runToPosition(double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
        resetEncoders();
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // =====================
    // SERVO METHODS
    // =====================

    private void rotateServoStep() {
        servoStep++;
        if (servoStep >= SERVO_STEPS) servoStep = 0;
        double servoPosition = servoStep / (double) SERVO_STEPS;
        step = (servoPosition - servo_position) / 10;
        servo_position = servo_position + step;
        spinning_pad_discrete.setPosition(servo_position);
    }

    // =====================
    // APRILTAG METHODS
    // =====================

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
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
                        "ID %d | XYZ %.1f %.1f %.1f",
                        d.id, d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
            } else {
                telemetry.addLine(String.format("ID %d | Unknown", d.id));
            }
        }
    }
}
