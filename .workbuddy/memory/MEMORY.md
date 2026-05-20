# Unitree R1 展厅控制项目 - 长期记忆

## 工具路径记录

### ffmpeg (音频转换工具)
- **路径**: `D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffmpeg.exe`
- **版本**: ffmpeg 8.1.1-essentials_build
- **用途**: MP3转PCM格式（16-bit, mono, 16000Hz）
- **安装日期**: 2026-05-18
- **状态**: ✅ 已配置，可直接使用

### edge-tts (文本转语音)
- **环境**: base conda环境 (`C:\Users\Zirui\miniconda3\python.exe`)
- **可用音色**: 
  - `zh-CN-XiaoxiaoNeural` - 默认女声
  - `zh-CN-YunxiNeural` - 默认男声
  - `zh-CN-liaoning-XiaobeiNeural` - **辽宁东北话女声** (推荐使用)
- **状态**: ✅ 已安装，可直接使用

---

## 项目约定

### 音频文件格式要求 (Unitree R1)
- **格式**: Raw PCM (无WAV头)
- **位深度**: 16-bit signed (s16le)
- **声道**: 单声道 (mono)
- **采样率**: 16000 Hz
- **文件扩展名**: `.pcm`

### 代码修改规范
- 修改 `show.cpp` 中的 `SCENES` 数组时，保持代码结构不变
- 音频文件路径使用绝对路径: `/home/unitree/voice_pack/audio_xxx/`
- 文本内容使用中文，多个文本用 `|` 分隔

---

## 重要提醒

1. **不要删除 ffmpeg 文件夹** - 已配置好，可重复使用
2. **py311环境已删除** - 不需要Python 3.11，base环境的Python 3.13可用
3. **生成的音频文件位置**: `D:\Unitree-R1-迎宾\voice_pack\audio_morning\`

---

## R1 SDK 记录

### SDK路径
- **最新SDK**: `C:\Users\Zirui\Desktop\R1最新SDK\unitree_sdk2-main\`
- **R1专用示例**: `example/r1/` （注意不是 example/g1/）
- **G1示例可参考但不能直接用**: R1只有26个电机，G1有29个

### R1 vs G1 关键差异
- R1: 26电机（含HEAD_PITCH/HEAD_YAW），G1: 29电机（无头部）
- R1手臂: 5电机/臂，G1: 7电机/臂（29DOF）
- R1 LocoClient 功能更少（无Squat/Sit/WaveHand/ShakeHand）
- R1 Kp/Kd 值不同（腿200/3, 臂100/2）

### R1 手臂动作列表（实测数据）
- 完整列表: `D:\Unitree-R1-迎宾\R1手臂动作完整列表.md`
- **已验证**: 99(release), 23(right_hand_up), 25(wave_under_head), 26(wave_above_head), 33(right_hand_on_heart)
- **重要**: ExecuteAction()是阻塞的，需等待动作完成才返回
- **最长动作**: ID 26=7.2秒, **最短**: ID 99=0.3秒
