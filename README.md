# HeatBench

HeatBench 是一个 ESP32-S3 网页恒温加热台项目。设备通过 MAX6675 读取 K 型热电偶，通过 SSR 控制加热，并提供本地 Wi-Fi 控制页面。

## 新手快速开始

1. 准备 ESP32-S3 开发板、HeatBench 低压转接板、MAX6675 模块和 K 型热电偶。状态灯为 WS2812 灯带/灯珠，默认数量是 7 颗。
2. 按照[低压接线手册](接线手册.md)接好传感器和状态灯。第一次启动只用 USB 给开发板供电，先不要连接任何 220 V 线路、SSR 市电端或加热板。
3. 在电脑安装并打开 Espressif ESP-IDF **6.1** 终端，进入本仓库目录。首次构建时，ESP-IDF Component Manager 会按 `dependencies.lock` 获取组件。
4. 构建并烧录：

   ```powershell
   idf.py set-target esp32s3
   idf.py build
   idf.py -p COMxx flash monitor
   ```

   把 `COMxx` 换成设备管理器中 ESP32-S3 实际使用的串口。也可以先只运行 `idf.py build` 编译，不烧录。

5. 启动后，在手机或电脑的 Wi-Fi 列表中连接 `HeatBench-XXXX` 热点。`XXXX` 是设备 MAC 地址末两字节。默认密码是 `heater1234`。
6. 打开浏览器访问 <http://192.168.4.1>。先确认页面显示传感器采样正常，再进行低压功能检查。

本仓库忽略本机生成的 `sdkconfig`；干净构建会使用 `sdkconfig.defaults` 和项目 Kconfig 默认值。热点默认密码 `heater1234` 定义在 `main/Kconfig.projbuild`，通常不需要新手手动设置。

## 仓库内容

- `main/`：ESP-IDF 固件源码和网页界面。
- `接线手册.md`：排针方向、逐线连接表和低压检查步骤。
- 底座打印文件 [PRINT_01_Base.stl](3d-model/PRINT_01_Base.stl)
- 顶盖打印文件 [PRINT_02_Lid.stl](3d-model/PRINT_02_Lid.stl)
