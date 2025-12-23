/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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

    // ----- DISCRETE SERVO CONTROL -----
    int servoStep = 8;                // 0,1,2
    final int SERVO_STEPS = 15;         // 360 / 120

    double servo_position=0.5;
    double step=0;

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

        drivetrain = new DriveTrain(
                hardwareMap.get(DcMotorEx.class, "M1"),
                hardwareMap.get(DcMotorEx.class, "M2"),
                hardwareMap.get(DcMotorEx.class, "M3"),
                hardwareMap.get(DcMotorEx.class, "M4")
        );

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad curGamepad1 = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {

            prevGamepad1.copy(curGamepad1);
            curGamepad1.copy(gamepad1);

            // Intake
            if (curGamepad1.a) intakeMotor.setPower(1);
            else intakeMotor.setPower(0);

            // Throwing
            if (curGamepad1.b) throwingMotor.setPower(1);
            else throwingMotor.setPower(0);

            // Bruce manual control
            bruce.setPower(curGamepad1.right_stick_x * -1);

            if (curGamepad1.right_stick_x == 0) {
                bruce.setPower(0);
            }

            // -------- SERVO DISCRETE ROTATION --------

            // Clockwise
            if (curGamepad1.dpad_right && !prevGamepad1.dpad_right) {
                servoStep++;
                if (servoStep >= SERVO_STEPS) servoStep = SERVO_STEPS;
            }

            // Counter-clockwise
            if (curGamepad1.dpad_left && !prevGamepad1.dpad_left) {
                servoStep--;
                if (servoStep < 0) servoStep = 0;
            }

            double servoPosition = servoStep / (double) SERVO_STEPS;
            step=(servoPosition-servo_position)/10;
            servo_position=servo_position+step;
            spinning_pad_discrete.setPosition(servo_position);

            // -------- APRILTAG AUTO TURN --------

            if (!aprilTag.getDetections().isEmpty()) {

                AprilTagDetection tag = aprilTag.getDetections().get(0);
                double error = (tag.center.x - 320) / 320.0;

                error = Math.max(-1, Math.min(1, error));

//                double turn = error * 0.8;
//                if (Math.abs(error) < 0.05) turn = 0;
//                turn = Math.max(-0.4, Math.min(0.4, turn));

                bruce.setPower(-error);

            } else {
                drivetrain.drive(gamepad1, 1);
            }

            // -------- TELEMETRY --------

            telemetry.addData("Servo Step", servoStep);
            telemetry.addData("Servo Angle (deg)", servoStep * 120);
            telemetry.addData("Servo Position", "%.3f", servoPosition);

            telemetryAprilTag();
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
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
                        "ID %d | XYZ %.1f %.1f %.1f",
                        d.id, d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
            } else {
                telemetry.addLine(String.format("ID %d | Unknown", d.id));
            }
        }
    }
}
