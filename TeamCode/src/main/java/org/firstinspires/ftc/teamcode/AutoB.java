package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@Autonomous(
        name = "AutoB"
)
public class AutoB extends LinearOpMode {
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
        this.TL.setDirection(Direction.REVERSE);
        this.BL.setDirection(Direction.REVERSE);
        this.waitForStart();
        if (this.opModeIsActive()) {
            powerEach(-0.7,0.7,-0.7,0.7,400);
            powerEach(-1D,-1D,-1D,-1D,1200);
            grab1.setPosition(0);
            grab2.setPosition(1);
            powerEach(z,z,z,z,800);
            powerEach(1D,1D,1D,1D,1200);
            powerEach(0.7,-0.7,-0.7,0.7,1200);
            grab1.setPosition(1);
            grab2.setPosition(0);
            sleep(400);
            powerEach(1D,1D,1D,1D,1200);
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
