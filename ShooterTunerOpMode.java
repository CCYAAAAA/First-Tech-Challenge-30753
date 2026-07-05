package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter Tuner Vision", group = "Test")
public class ShooterTunerOpMode extends OpMode {

    /* ===================== 常量 ===================== */

    private static final int TARGET_TAG_ID = 2;

    private static final double TAG_REAL_WIDTH_MM = 165.0;
    private static final double FOCAL_LENGTH = 309.0;

    private static final double DIST_ALPHA = 0.25;   // 距离滤波
    private static final double TURN_SENSITIVITY = 0.7;
    private static final double STRAFE_CORRECTION = 1.1;

    /* ===================== 硬件 ===================== */

    private DcMotor frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive;
    private DcMotor shooterUpMotor, shooterDownMotor;
    private DcMotor intakeMotor, indexerMotor;
    private VoltageSensor voltageSensor;
    private HuskyLens huskyLens;

    /* ===================== 状态变量 ===================== */

    private double shooterRatio = 0.65;     // 上 / 下 电机比例
    private double targetPower = 0.85;      // 仅显示用
    private double lockedPower = 0;

    private double filteredDistanceMM = -1;
    private double lockedDistanceMM = -1;

    private boolean shooterLocked = false;
    private boolean lastTrigger = false;

    /* ===================== 初始化 ===================== */

    @Override
    public void init() {

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "rear_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right");

        shooterUpMotor   = hardwareMap.get(DcMotor.class, "shooter_up");
        shooterDownMotor = hardwareMap.get(DcMotor.class, "shooter_down");
        intakeMotor      = hardwareMap.get(DcMotor.class, "intake");
        indexerMotor     = hardwareMap.get(DcMotor.class, "indexer");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        shooterUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /* ===================== 主循环 ===================== */

    @Override
    public void loop() {
        updateAuxMotors();
        updateVisionDistance();
        updateShooterControl();
        driveBaseControl();
        updateTelemetry();
    }

    /* ===================== 视觉距离 ===================== */

    private void updateVisionDistance() {

        HuskyLens.Block[] blocks = huskyLens.blocks();

        for (HuskyLens.Block block : blocks) {
            if (block.id == TARGET_TAG_ID && block.width > 0) {

                double aspect = (double) block.width / block.height;
                if (aspect < 0.8 || aspect > 1.2) return;

                double rawDistance =
                        (TAG_REAL_WIDTH_MM * FOCAL_LENGTH) / block.width;

                if (filteredDistanceMM < 0) {
                    filteredDistanceMM = rawDistance;
                } else {
                    filteredDistanceMM =
                            DIST_ALPHA * rawDistance +
                                    (1 - DIST_ALPHA) * filteredDistanceMM;
                }
                return;
            }
        }
    }

    /* ===================== 射击控制 ===================== */

    private void updateShooterControl() {

        boolean trigger = gamepad1.right_trigger > 0.1;

        // 右扳机上升沿：锁定距离 & 功率
        if (trigger && !lastTrigger) {
            shooterLocked = true;
            lockedDistanceMM = filteredDistanceMM;
            lockedPower = computeShooterPower(
                    lockedDistanceMM,
                    voltageSensor.getVoltage()
            );
        }

        if (!trigger) {
            shooterLocked = false;
            shooterUpMotor.setPower(0);
            shooterDownMotor.setPower(0);
        } else {
            shooterUpMotor.setPower(lockedPower * shooterRatio);
            shooterDownMotor.setPower(lockedPower);
        }

        lastTrigger = trigger;

        // 肩键微调上下飞轮比例
        if (gamepad1.right_bumper) shooterRatio += 0.01;
        if (gamepad1.left_bumper)  shooterRatio -= 0.01;
        shooterRatio = Range.clip(shooterRatio, 0.4, 1.0);
    }

    /* ===================== 功率模型（中段强化·激进版） ===================== */

    /* ===================== 功率模型（中段强化 + 低压激进） ===================== */

    private double computeShooterPower(double distMM, double voltage) {

        distMM = Range.clip(distMM, 120, 260);

        // ===== 距离 → 功率（你当前激进中段系数） =====
        double base =
                0.0000165 * distMM * distMM +
                        0.0034    * distMM +
                        0.515;

        // ===== 电压补偿（低压加速） =====
        double voltComp;
        if (voltage >= 13.2) {
            voltComp = (13.5 - voltage) * 0.035;   // 正常区
        } else {
            voltComp = (13.5 - voltage) * 0.065;   // 低压猛补
        }

        return Range.clip(base + voltComp, 0.82, 1.0);
    }



    /* ===================== 进弹 / 推弹 ===================== */

    private void updateAuxMotors() {

        // Intake
        if (gamepad1.a) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.b) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0);
        }

        // Indexer
        if (gamepad1.x) {
            indexerMotor.setPower(1.0);
        } else if (gamepad1.y) {
            indexerMotor.setPower(-1.0);
        } else {
            indexerMotor.setPower(0);
        }
    }

    /* ===================== 底盘 ===================== */

    private void driveBaseControl() {

        double drive  = gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y * STRAFE_CORRECTION;
        double turn   = gamepad1.right_stick_x * TURN_SENSITIVITY;

        double fl = drive + strafe + turn;
        double fr = drive - strafe + turn;
        double rl = drive - strafe - turn;
        double rr = drive + strafe - turn;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(rl), Math.abs(rr))));

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            rl /= max;
            rr /= max;
        }

        frontLeftDrive.setPower(fl);
        rearLeftDrive.setPower(rl);
        frontRightDrive.setPower(fr);
        rearRightDrive.setPower(rr);
    }

    /* ===================== 显示 ===================== */

    private void updateTelemetry() {

        telemetry.addData("Tag ID", TARGET_TAG_ID);

        telemetry.addData("Distance",
                shooterLocked ?
                        "%.1f mm (LOCKED)" :
                        "%.1f mm (FILTERED)",
                shooterLocked ? lockedDistanceMM : filteredDistanceMM);

        telemetry.addData("Locked Power", "%.3f", lockedPower);
        telemetry.addData("Shooter Ratio (Up/Down)", "%.2f", shooterRatio);
        telemetry.addData("Voltage", "%.2f V", voltageSensor.getVoltage());

        telemetry.update();
    }
}
