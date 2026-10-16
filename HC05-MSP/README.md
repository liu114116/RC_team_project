# LFish · 青鳍 0.2.0

HC-05 手机飞控项目，适配可安装安卓 APK 的 HarmonyOS 4.2 / Android 8+。

## 当前版本

- `LFish-0.2.0.apk`：新版安装包，包名沿用 cn.rcproject.msp，签名沿用旧版调试密钥。
- `source/`：完整源码、本地界面资源、构建脚本和调试签名。调试密钥口令 android，仅用于本项目开发，不作为生产密钥。
- `assets/lfish-icon.png`：鱼形机身与四旋翼融合图标的 1024px 原图。
- `preview/`：电脑预览，与 APK 共用页面；运行 Python HTTP 服务后打开 app.html。
- `使用说明.md`：连接、功能边界和后续验证步骤。

新版提供连接入口、控制和连接状态两个页面；没有独立调参页，没有操纵归零按钮。校准、PID、电机和模式配置在 Betaflight 中进行。双摇杆松手回中，左杆升降/转向，右杆前后/左右。

**当前控制页是交互演示。真实蓝牙连接仅查询遥测，不发送 RC、解锁、模式修改或电机指令。定高飞行控制尚未完成。**

固件已确认提供 BARO、ANGLE、RX_MSP 及 alt_hold 参数。理论上具备气压定高基础，但定高启用时记录当时油门；需完成起飞/定高衔接、升降限幅、失控和实机验证后才能开放操纵。BARO+ANGLE 不等于水平定点，当前不承诺水平悬停。

## 构建

使用 JDK、Android platform android.jar 和 Windows build-tools：

```powershell
./source/build.ps1 -Jdk '<JDK目录>' -BuildTools '<build-tools目录>' -AndroidJar '<android.jar路径>'
```

产物位于 source/build/HC05-MSP-debug.apk。首次构建时将 source/debug-signing.jks 复制到 source/build/debug.jks 以沿用签名。

## 验证

已通过 Java 编译、MSP 帧解析/校验和/通道映射单元测试、DEX 构建、APK v2/v3 签名验证，以及电脑浏览器连接/演示控制/状态页面检查。未完成手机安装、Android WebView 真机显示、HC-05 实机通信或飞行测试。

## 图标

内置图像生成工具未返回可保存图像，最终图标用 Pillow 本地绘制，已嵌入启动图标与连接页。设计：深青绿背景、浅色鱼形机身、尾鳍和四个旋翼圆环，呼应 LFish 与飞控功能。图形生成脚本见 source/make_icon.py。最初生成提示方向为“白色四旋翼、深青绿色背景、精简立体应用图标，无文字”。
