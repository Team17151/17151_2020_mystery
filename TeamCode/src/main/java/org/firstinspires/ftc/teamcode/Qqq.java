package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;

@TeleOp(name = "Qqq", group = "")
public class Qqq extends LinearOpMode {

    private VuforiaSkyStone vuforiaSkyStone;
    private DcMotor TR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor TL;
    private Servo clawboi;
    VuforiaBase.TrackingResults vuforiaResults;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        vuforiaSkyStone = new VuforiaSkyStone();
        this.TR = (DcMotor)this.hardwareMap.dcMotor.get("TR");
        this.TL = (DcMotor)this.hardwareMap.dcMotor.get("TL");
        this.BL = (DcMotor)this.hardwareMap.dcMotor.get("BL");
        this.BR = (DcMotor)this.hardwareMap.dcMotor.get("BR");
        clawboi = hardwareMap.servo.get("clawboi");
        Double z = 0D;
        this.TR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize Vuforia
        telemetry.addData("Status", "Initializing Vuforia. Please wait...");
        telemetry.update();
        initVuforia();
        // Activate here for camera preview.
        vuforiaSkyStone.activate();
        telemetry.addData(">>", "Vuforia initialized, press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            boolean endloop = false;
            double zangle;
            TR.setPower(-0.4);
            TL.setPower(0.4D);
            BR.setPower(0.8D);
            BL.setPower(-0.8);
            sleep(500);
            while (opModeIsActive() && !endloop) {
                // Are the targets visible?
                // (Note we only process first visible target).
                if (isTargetVisible("Stone Target")) {
                    processTarget();
                    TR.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    TL.setPower(0);
                    endloop = true;
                } else {
                    telemetry.addData("No Targets Detected", "Targets are not visible.");
                    TR.setPower(.23);
                    BR.setPower(.23);
                    BL.setPower(.23);
                    TL.setPower(.23);
                    sleep(350);
                    TR.setPower(z);
                    BR.setPower(z);
                    BL.setPower(z);
                    TL.setPower(z);
                    sleep(400);
                }
                telemetry.update();
            }
            double boold;
            zangle = Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2));
            while (isTargetVisible("Stone Target") && opModeIsActive() && ((-82 < zangle) || (-98 > zangle))) {
                zangle = Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2));
                boold = Math.signum(zangle + 90);
                TR.setPower(.2 * -boold);
                TL.setPower(.2 * boold);
                BR.setPower(.2 * -boold);
                BL.setPower(.2 * boold);
                processTarget();
                telemetry.update();
            }
            TR.setPower(z);
            BR.setPower(z);
            BL.setPower(z);
            TL.setPower(z);
            sleep(200);
            zangle = Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "IN"), 2));
            while (isTargetVisible("Stone Target") && opModeIsActive() && ((-1 < zangle) || (-1.8 > zangle))) {
                zangle = Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "IN"), 2));
                boold = Math.signum(zangle + 1);
                TR.setPower(boold * .2);
                TL.setPower(boold * .2);
                BL.setPower(boold * .2);
                BR.setPower(boold * .2);
                sleep(75);
                TR.setPower(z);
                BR.setPower(z);
                BL.setPower(z);
                TL.setPower(z);
                sleep(75);
                processTarget();
                telemetry.update();
            }
            TR.setPower(-0.4);
            TL.setPower(0.4D);
            BR.setPower(0.8D);
            BL.setPower(-0.8);
            while (isTargetVisible("Stone Target") && opModeIsActive()) {
                TR.setPower(-0.5);
                TL.setPower(0.4D);
                BR.setPower(0.7D);
                BL.setPower(-0.8);
                sleep(100);
                TR.setPower(z);
                BR.setPower(z);
                BL.setPower(z);
                TL.setPower(z);
                sleep(100);
            }
            clawboi.setPosition(.6);
            sleep(1000);
            TR.setPower(0.4);
            TL.setPower(-0.4D);
            BR.setPower(-0.8D);
            BL.setPower(0.8);
            sleep(500);
        }
        // Don't forget to deactivate Vuforia.
        vuforiaSkyStone.deactivate();
        vuforiaSkyStone.close();
    }

    /**
     * Describe this function...
     */
    private void initVuforia() {
        // Initialize using external web camera.
        vuforiaSkyStone.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "Webcam 1", // webcamCalibrationFilename
                true, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
    }

    /**
     * Check to see if the target is visible.
     */
    private boolean isTargetVisible(String trackableName) {
        boolean isVisible;

        // Get vuforia results for target.
        vuforiaResults = vuforiaSkyStone.track(trackableName);
        // Is this target visible?
        if (vuforiaResults.isVisible) {
            isVisible = true;
        } else {
            isVisible = false;
        }
        return isVisible;
    }

    /**
     * This function displays location on the field and rotation about the Z
     * axis on the field. It uses results from the isTargetVisible function.
     */
    private void processTarget() {
        // Display the target name.
        telemetry.addData("Target Detected", vuforiaResults.name + " is visible.");
        telemetry.addData("X (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.x, "IN"), 2)));
        telemetry.addData("Y (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "IN"), 2)));
        telemetry.addData("Z (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.z, "IN"), 2)));
        telemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2)));
    }

    /**
     * By default, distances are returned in millimeters by Vuforia.
     * Convert to other distance units (CM, M, IN, and FT).
     */
    private double displayValue(float originalValue, String units) {
        double convertedValue;

        // Vuforia returns distances in mm.
        if (units.equals("CM")) {
            convertedValue = originalValue / 10;
        } else if (units.equals("M")) {
            convertedValue = originalValue / 1000;
        } else if (units.equals("IN")) {
            convertedValue = originalValue / 25.4;
        } else if (units.equals("FT")) {
            convertedValue = (originalValue / 25.4) / 12;
        } else {
            convertedValue = originalValue;
        }
        return convertedValue;
    }
}