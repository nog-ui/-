// 引入 MPU6050 传感器的库（Tockn 版本，封装了姿态解算）
// 该库可以直接获取角度（Angle），对新手非常友好
#include <MPU6050_tockn.h>

// 引入 I2C 通信库
#include <Wire.h>

// 创建一个 MPU6050 对象，使用 Wire（I2C）通信
MPU6050 mpu6050(Wire);

void setup() {
  // 初始化串口通信，波特率 115200
  // 用于在串口监视器中查看传感器数据
  Serial.begin(115200);

  // 初始化 I2C
  // 对于 ESP32C3：
  // 第一个参数 4  -> SDA 引脚
  // 第二个参数 5  -> SCL 引脚
  Wire.begin(4, 5);

  // 初始化 MPU6050
  // 包括检测设备是否存在、配置寄存器等
  mpu6050.begin();

  // 计算陀螺仪的零偏（非常重要）
  // true 表示在串口中显示校准过程提示
  // 校准时请保持传感器静止！
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  // 更新 MPU6050 数据
  // 必须在 loop() 中不断调用
  // 内部会读取加速度计和陀螺仪数据，并进行姿态融合计算
  mpu6050.update();

  // 输出 X 轴角度（通常对应前后倾斜，Pitch）
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());

  // 输出 Y 轴角度（通常对应左右倾斜，Roll）
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());

  // 输出 Z 轴角度（通常对应旋转角，Yaw）
  // 注意：Z 轴角度会随时间漂移，这是陀螺仪的固有问题
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());

  // 可根据需要加入延时（如 10ms）
  delay(10);
}
