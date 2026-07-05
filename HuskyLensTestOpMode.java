package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "HuskyLens Locator Test", group = "Sensor")
public class HuskyLensTestOpMode extends LinearOpMode {

    private HuskyLens huskyLens;

    // --- 视觉测距参数 (需要根据实际校准) ---
    // 1. 拿尺子量一下你的 AprilTag 的黑色边框的实际宽度 (毫米)
    // FTC 比赛常用的 Tag 通常是 2英寸 (约50.8mm) 或 4英寸 (101.6mm)
    private static final double TAG_REAL_WIDTH_MM = 165;

    // 2. 焦距系数 (需要校准)
    // 校准方法：把 Tag 放在距离摄像头正好 200mm (20cm) 的地方。
    // 读取 telemetry 上显示的 "Pixel Width"。
    // 焦距系数 = (200mm * 像素宽度) / 实际宽度mm
    // 默认估算值：假如 200mm 处宽度是 60像素 -> 200*60/50.8 = 236
    private static final double FOCAL_LENGTH_CONSTANT = 309;

    @Override
    public void runOpMode() {
        // 1. 初始化硬件
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // 2. 设置算法为 TAG_RECOGNITION (AprilTag识别)
        // 确保你的 HuskyLens 屏幕上显示的是 "Tag Recognition" 模式
        // 如果不是，可以通过机身按键切换，或者用代码强制切换（有时代码切换不稳定，建议手动切好）
        if (!huskyLens.knock()) {
            telemetry.addData("Warning", "HuskyLens not responding!");
            telemetry.update();
        } else {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        telemetry.addData("Status", "HuskyLens Ready");
        telemetry.addData("Mode", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        // 设置读取频率限制，防止 I2C 总线堵塞
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset(); // 每秒读取一次 (可以调快，例如 100ms)

            // 获取当前画面所有的 Block (识别到的物体)
            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Block Count", blocks.length);

            if (blocks.length > 0) {
                for (int i = 0; i < blocks.length; i++) {
                    HuskyLens.Block block = blocks[i];

                    // --- 计算距离和位置 ---

                    // 1. 计算距离 (Distance)
                    // 公式：距离 = (实际宽度 * 焦距系数) / 画面像素宽度
                    double distMM = (TAG_REAL_WIDTH_MM * FOCAL_LENGTH_CONSTANT) / block.width;

                    // 2. 计算横向偏移 (X Offset)
                    // HuskyLens 分辨率通常为 320x240。中心点 X = 160。
                    // 负数表示在左边，正数表示在右边
                    int centerX = 160;
                    int offsetPixels = block.x - centerX;

                    // 利用相似三角形计算实际横向偏移距离 (mm)
                    // 偏移mm = (偏移像素 * 距离) / 焦距系数 (近似)
                    double offsetX_MM = (offsetPixels * distMM) / FOCAL_LENGTH_CONSTANT;

                    // --- 显示数据 ---
                    telemetry.addData("Tag ID", block.id);
                    telemetry.addData("  - Size(WxH)", "%dx%d", block.width, block.height);
                    telemetry.addData("  - Pos(X,Y)", "%d, %d", block.x, block.y);
                    telemetry.addData("  - Distance", "%.1f mm (%.1f in)", distMM, distMM / 25.4);
                    telemetry.addData("  - Offset X", "%.1f mm", offsetX_MM);

                    // 简单的定位逻辑示例：
                    if (block.id == 1) {
                        telemetry.addData("Location", "Blue Alliance Left");
                    } else if (block.id == 2) {
                        telemetry.addData("Location", "Blue Alliance Center");
                    }
                }
            } else {
                telemetry.addData("Status", "No Tag Detected");
            }

            telemetry.update();
        }
    }
}