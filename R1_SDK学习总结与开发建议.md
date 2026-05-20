# Unitree R1 SDK 学习总结与开发建议

**学习时间**: 2026-05-20  
**SDK版本**: unitree_sdk2-main (最新版)  
**电机SDK**: unitree_actuator_sdk-main  

---

## 📚 学习总结

### 1️⃣ **R1 SDK 实际功能（基于 `example/r1/` 目录）**

#### **高层API（High-level API）**

| 功能 | 示例文件 | R1专属 | 说明 |
|------|----------|--------|------|
| **locomotion控制** | `r1_loco_client_example.cpp` | ✅ R1专属 | 行走、站立、速度控制 |
| **手臂动作** | G1共享 | ❌ 复用G1 | 执行预定义动作（ID: 23-99）|
| **音频播放** | G1共享 | ❌ 复用G1 | PCM音频流播放 |

#### **低层API（Low-level API）**

| 功能 | 示例文件 | R1专属 | 说明 |
|------|----------|--------|------|
| **电机控制** | `r1_ankle_swing_example.cpp` | ✅ R1专属 | **26电机**控制 |

---

### 2️⃣ **R1 vs G1 关键差异**

| 特性 | R1 | G1 |
|------|----|----|
| **电机数量** | **26** | 29 |
| **头部电机** | ✅ HEAD_PITCH + HEAD_YAW | ❌ 无 |
| **手臂电机** | 5/臂 (shoulder_pitch/roll/yaw + elbow + wrist_roll) | 7/臂 (29DOF) |
| **膝关节电机** | 1/腿 | 1/腿 |
| **locomotion功能** | Start, StandUp, SetVelocity, Move, StopMove | + Squat, Sit, WaveHand, ShakeHand |
| **刚度值** | 腿: 200, 臂: 100, 头: 50/10 | 腿: 40-100, 臂: 40 |
| **阻尼值** | 腿: 3, 臂: 2, 头: 2/0.1 | 腿: 1-2, 臂: 1 |

---

### 3️⃣ **R1 26电机关节索引**

```
关节ID | 名称               | 用途
-------|-------------------|------------------
0-5    | 左腿 (6个)        | HipPitch/Roll/Yaw, Knee, AnklePitch/Roll
6-11   | 右腿 (6个)        | HipPitch/Roll/Yaw, Knee, AnklePitch/Roll
12     | 腰部Roll          | 左右倾斜
13     | 腰部Yaw           | 左右转动
14-18  | 左臂 (5个)        | ShoulderPitch/Roll/Yaw, Elbow, WristRoll
19-23  | 右臂 (5个)        | ShoulderPitch/Roll/Yaw, Elbow, WristRoll
24     | 头部Pitch          | 上下点头 ⭐ R1专属
25     | 头部Yaw            | 左右转头 ⭐ R1专属
```

---

## 💡 开发建议（基于R1实际功能）

### 🎯 **方案A：利用头部电机增加互动体验**（推荐优先）

**R1独有功能：HEAD_PITCH 和 HEAD_YAW**

**改进方向：**
- 问候时头部微微点头
- 引导参观时转头指向展示区
- 与访客交互时跟踪人脸

**代码示例：**
```cpp
void NodHead(float angle_deg) {
    MotorCommand cmd;
    
    // 设置R1 Kp/Kd（从r1_ankle_swing_example.cpp复制）
    for (int i = 0; i < R1_NUM_MOTOR; ++i) {
        cmd.kp[i] = R1_Kp[i];
        cmd.kd[i] = R1_Kd[i];
    }
    
    // 点头动作
    cmd.q_target[HEAD_PITCH] = angle_deg * M_PI / 180.0;
    cmd.q_target[HEAD_YAW] = 0.0;
    
    motor_command_buffer_.SetData(cmd);
}
```

---

### 🎯 **方案B：移动迎宾**

**R1 LocoClient 支持：**
- `Start()` - 启动 locomotion
- `SetVelocity(vx, vy, omega, duration)` - 设置速度（0.5 m/s往前走）
- `Move(vx, vy, omega)` - 持续移动
- `StopMove()` - 停止移动
- `StandUp()` - 站立

**改进方向：**
- 按键触发：R1走向访客 → 停下 → 问候 → 返回

**代码示例：**
```cpp
void MobileGreeting() {
    // 1. 启动locomotion
    loco_client->Start();
    
    // 2. 往前走2秒
    loco_client->SetVelocity(0.3, 0.0, 0.0, 2.0);
    
    // 3. 执行问候
    PlayScene(0);
    
    // 4. 后退2秒
    loco_client->SetVelocity(-0.3, 0.0, 0.0, 2.0);
    
    // 5. 停止
    loco_client->StopMove();
}
```

---

### 🎯 **方案C：自定义手臂动作**

**R1手臂电机：**
- LeftShoulderPitch (14) / RightShoulderPitch (19)
- LeftShoulderRoll (15) / RightShoulderRoll (20)
- LeftShoulderYaw (16) / RightShoulderYaw (21)
- LeftElbow (17) / RightElbow (22)
- LeftWristRoll (18) / RightWristRoll (23)

**改进方向：**
- 平滑挥手（比预定义ID更自然）
- 引导动作（指向展示区）
- 双手协调动作

**代码示例：**
```cpp
void SmoothWaveHand() {
    MotorCommand cmd;
    
    // 设置手臂Kp/Kd
    for (int i = 14; i <= 23; ++i) {
        cmd.kp[i] = 100;
        cmd.kd[i] = 2;
    }
    
    // 挥手轨迹
    for (int t = 0; t < 2000; t += 10) {
        double phase = (double)t / 2000 * 2 * M_PI;
        cmd.q_target[RightShoulderPitch] = 0.5 * sin(phase);
        cmd.q_target[RightShoulderRoll] = 0.3 * sin(phase * 2);
        cmd.q_target[RightElbow] = -0.8;
        
        motor_command_buffer_.SetData(cmd);
        usleep(10000);  // 10ms
    }
}
```

---

### 🎯 **方案D：R1专属动作组合**

**结合头部+手臂+腿部：**
- 按键1：点头 + 挥手（问候）
- 按键2：转头 + 指向（引导）
- 按键3：鞠躬 + 后退（感谢）

---

## 📋 下一步行动

| 方案 | 难度 | 效果 | R1专属 |
|------|------|------|--------|
| A. 头部点头互动 | ⭐ 简单 | ⭐⭐⭐ 很好 | ✅ |
| B. 移动迎宾 | ⭐⭐ 中等 | ⭐⭐⭐ 很好 | - |
| C. 自定义手臂动作 | ⭐⭐ 中等 | ⭐⭐ 好 | - |
| D. 动作组合 | ⭐⭐⭐ 高 | ⭐⭐⭐ 很好 | ✅ |

---

## ❓ 你的问题

1. 你想先试哪个方案？
2. 你的R1是否可以走动（有 locomotion 功能）？
3. 你对头部电机控制（点头、转头）感兴趣吗？
