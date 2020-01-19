package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@Autonomous(
        name = "AutoBl"
)
public class AutoBl extends LinearOpMode {
    private DcMotor arm;
    private DcMotor TR;
    private DcMotor TL;
    private DcMotor BL;
    private DcMotor BR;
    private Servo grab1;
    private Servo grab2;

    public void runOpMode() {
        this.TR = (DcMotor)this.hardwareMap.dcMotor.get("TR");
        this.TL = (DcMotor)this.hardwareMap.dcMotor.get("TL");
        this.BL = (DcMotor)this.hardwareMap.dcMotor.get("BL");
        this.BR = (DcMotor)this.hardwareMap.dcMotor.get("BR");
        grab1 = hardwareMap.servo.get("grab1");
        grab2 = hardwareMap.servo.get("grab2");
        grab1.setPosition(1);
        grab2.setPosition(0);

        Double z = 0D;
        this.TR.setDirection(Direction.REVERSE);
        this.BR.setDirection(Direction.REVERSE);
        this.waitForStart();
        if (this.opModeIsActive()) {
            powerEach(-1D,-1D,-1D,-1D,60);
            powerEach(0.7,-0.7,0.7,-0.7,200);
            powerEach(-1D,-1D,-1D,-1D,440);
            grab1.setPosition(0);
            grab2.setPosition(1);
            powerEach(z,z,z,z,700);
            powerEach(1D,1D,1D,1D,600);
            powerEach(-0.6,0.6,-1D,1D,1500);
            powerEach(-1D,-0.8D,-1D,-0.8D,1500);
            grab1.setPosition(1);
            grab2.setPosition(0);
            powerEach(z,z,z,z,400);
            powerEach(0.8,0.6,0.8,0.6,1000);
            powerEach(z,z,z,z,0);
        }

    }
    private void powerEach(Double TRink, Double TLink, Double BLink, Double BRink, Integer tim) {
        this.TR.setPower(TRink);
        this.TL.setPower(TLink);
        this.BL.setPower(BLink);
        this.BR.setPower(BRink);
        this.sleep(tim);
    }
}
