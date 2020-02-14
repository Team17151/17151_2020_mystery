package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.SensorMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(
        name = "AutoBlo"
)
public class AutoBlo extends LinearOpMode {
    private DcMotor arm;
    private DcMotor TR;
    private DcMotor TL;
    private DcMotor BL;
    private DcMotor BR;
    private Servo grab1;
    private Servo grab2;
    private BNO055IMU imu;
    private VoltageSensor voltageSensor;
    private ColorSensor veryvery;

    public void runOpMode() {
        this.TR = (DcMotor)this.hardwareMap.dcMotor.get("TR");
        this.TL = (DcMotor)this.hardwareMap.dcMotor.get("TL");
        this.BL = (DcMotor)this.hardwareMap.dcMotor.get("BL");
        this.BR = (DcMotor)this.hardwareMap.dcMotor.get("BR");
        grab1 = hardwareMap.servo.get("grab1");
        grab2 = hardwareMap.servo.get("grab2");
        grab1.setPosition(1);
        grab2.setPosition(0);
        veryvery = hardwareMap.get(ColorSensor.class, "veryvery");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        Double z = 0D;
        this.TR.setDirection(Direction.REVERSE);
        this.BR.setDirection(Direction.REVERSE);
        this.waitForStart();
        if (this.opModeIsActive()) {
            powerEach(-.35D,.35D,-.5D,.5D,1500);
            sleep(100);
            if (veryvery.red() < 280) {
                powerEach(-.7,.7,-1D,1D,300);
            }
            powerEach(z,z,z,z,0);
        }

    }
    private void powerEach(Double TRink, Double TLink, Double BLink, Double BRink, Integer tim) {
        this.TR.setPower(TRink);
        this.TL.setPower(TLink);
        this.BL.setPower(BLink);
        this.BR.setPower(BRink);
        this.sleep(tim);
        stabl();
    }
    private void stabl() {
        double gyro = -imu.getAngularOrientation().firstAngle;
        while (gyro < -10 || gyro > 10 && opModeIsActive()) {
            gyro = -imu.getAngularOrientation().firstAngle;
            TR.setPower(gyro / 50);
            TL.setPower(-gyro / 50);
            BL.setPower(-gyro / 50);
            BR.setPower(gyro / 50);
            telemetry.addData("geero",gyro);
            telemetry.update();
        }
        TR.setPower(0);
        TL.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
