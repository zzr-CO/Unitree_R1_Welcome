# Unitree R1 音频生成技术文档
## 使用 edge-tts + ffmpeg 生成PCM音频

---

## 📋 目录

1. [概述](#概述)
2. [技术栈](#技术栈)
3. [环境配置](#环境配置)
4. [完整流程](#完整流程)
5. [PCM格式详解](#pcm格式详解)
6. [代码示例](#代码示例)
7. [常用命令](#常用命令)
8. [故障排查](#故障排查)
9. [最佳实践](#最佳实践)

---

## 概述

本文档介绍如何使用 **edge-tts** (Microsoft Azure Text-to-Speech) 生成中文语音，并通过 **ffmpeg** 转换为 Unitree R1 机器人所需的 **PCM 音频格式**。

**适用场景：**
- ✅ 生成机器人迎宾/问候语音
- ✅ 需要东北话/方言音色
- ✅ 批量生成多版本音频
- ✅ 快速迭代音频内容

---

## 技术栈

### 1️⃣ edge-tts
- **简介**: Microsoft Azure TTS 的 Python 接口，免费使用
- **优点**: 
  - 音质好，支持多种中文音色
  - 支持方言（东北话、陕西话等）
  - 完全免费，无需API key
- **局限**: 需要联网使用

### 2️⃣ ffmpeg
- **简介**: 强大的音视频处理工具
- **用途**: 将 MP3 转换为 Raw PCM 格式
- **路径**: `D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffmpeg.exe`

### 3️⃣ Python 3.13 (base conda环境)
- **路径**: `C:\Users\Zirui\miniconda3\python.exe`
- **必需库**: `edge-tts` (已安装)

---

## 环境配置

### 检查 edge-tts 是否可用

```powershell
C:\Users\Zirui\miniconda3\python.exe -c "import edge_tts; print('OK')"
```

### 检查 ffmpeg 是否可用

```powershell
& "D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffmpeg.exe" -version
```

### 推荐音色列表

| 音色ID | 说明 | 适用场景 |
|--------|------|----------|
| `zh-CN-XiaoxiaoNeural` | 温柔女声（默认） | 通用场景 |
| `zh-CN-YunxiNeural` | 稳重男声 | 正式场合 |
| `zh-CN-liaoning-XiaobeiNeural` | **辽宁东北话女声** | 迎宾、问候（推荐）|
| `zh-CN-shandong-YunyangNeural` | 山东话男声 | 方言场景 |

**查看所有可用音色：**
```powershell
C:\Users\Zirui\miniconda3\python.exe -m edge_tts.list_voices
```

---

## 完整流程

### 流程图

```
┌─────────────────┐
│  准备文本内容    │
│ (东北话问候语)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ edge-tts 生成   │
│ MP3 文件        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ ffmpeg 转换     │
│ MP3 → PCM       │
│ (16-bit, mono,  │
│  16000Hz)       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 上传到机器人    │
│ /home/unitree/  │
│ voice_pack/      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 修改 show.cpp   │
│ SCENES 数组     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 重新编译部署    │
│ make & systemctl│
└─────────────────┘
```

### 步骤1: 生成 MP3 文件

**Python脚本示例：**

```python
import asyncio
import edge_tts

async def generate_mp3(text, output_path, voice="zh-CN-liaoning-XiaobeiNeural"):
    """生成MP3文件"""
    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(output_path)
    print(f"✓ 生成: {output_path}")

# 使用示例
text = "早啊！今儿个又是元气满满的一天！"
await generate_mp3(text, "output.mp3")
```

### 步骤2: 转换 MP3 为 PCM

**ffmpeg 命令：**

```powershell
$ffmpeg = "D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffmpeg.exe"
$mp3 = "input.mp3"
$pcm = "output.pcm"

& $ffmpeg -i $mp3 -f s16le -acodec pcm_s16le -ac 1 -ar 16000 $pcm -y
```

**参数详解：**

| 参数 | 说明 |
|------|------|
| `-i input.mp3` | 输入文件 |
| `-f s16le` | 输出格式：16-bit signed PCM |
| `-acodec pcm_s16le` | 音频编解码器 |
| `-ac 1` | 单声道 (mono) |
| `-ar 16000` | 采样率 16000 Hz |
| `-y` | 覆盖已存在的文件 |
| `output.pcm` | 输出文件 (Raw PCM，无WAV头) |

---

## PCM格式详解

### 什么是 Raw PCM？

**PCM (Pulse Code Modulation)** 是未压缩的原始音频数据，Unitree R1 机器人直接使用这种格式播放。

**关键特征：**
- ❌ **无文件头** (No header) - 不是WAV文件
- ✅ **原始采样数据** - 直接送给DAC播放
- ✅ **固定参数** - 16-bit, mono, 16000Hz

### 为什么不用WAV？

Unitree R1 的音频播放接口 (`ALSA`/`aplay`) 需要 Raw PCM 格式：
```cpp
// show.cpp 中的音频播放代码
system("aplay -f S16_LE -r 16000 -c 1 audio.pcm");
```

如果使用WAV文件，`aplay` 会把文件头当成音频数据播放，产生杂音。

### 参数对照表

| 参数 | 值 | ffmpeg 参数 |
|------|-----|--------------|
| 位深度 | 16-bit | `s16le` |
| 编码 | Signed PCM | `pcm_s16le` |
| 声道数 | 1 (Mono) | `-ac 1` |
| 采样率 | 16000 Hz | `-ar 16000` |
| 字节序 | Little-Endian | `le` in `s16le` |

---

## 代码示例

### 完整Python脚本

**文件**: `generate_northeast_xiaobei.py`

```python
"""
生成东北话早安问候PCM音频文件
音色: zh-CN-liaoning-XiaobeiNeural
输出: PCM格式 (16-bit, mono, 16000Hz)
"""

import asyncio
import edge_tts
import subprocess
import os

# 东北话早安问候文本
TEXTS = [
    "早啊！今儿个又是元气满满的一天，祝您工作顺心，啥事儿都顺顺利利的！",
    "清早儿好啊！哎妈呀，今儿个心情不错吧？带着好心情开启充实的一天！",
    "早啊！迎着晨光来上班啦？今儿个工作可得劲儿了，万事如意！",
    "早安呐～新的一天新的开始，给您加满油！工作轻松，啥都顺！",
]

# 输出目录
OUTPUT_DIR = r"D:\Unitree-R1-迎宾\voice_pack\audio_morning"
FFMPEG_PATH = r"D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffmpeg.exe"

async def generate_mp3(text, output_path, voice="zh-CN-liaoning-XiaobeiNeural"):
    """使用edge-tts生成MP3文件"""
    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(output_path)
    print(f"✓ 生成MP3: {os.path.basename(output_path)}")

def convert_mp3_to_pcm(mp3_path, pcm_path):
    """将MP3转换为PCM格式"""
    cmd = [
        FFMPEG_PATH,
        "-i", mp3_path,
        "-f", "s16le",
        "-acodec", "pcm_s16le",
        "-ac", "1",
        "-ar", "16000",
        pcm_path,
        "-y"
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"✓ 转换PCM: {os.path.basename(pcm_path)}")
        return True
    else:
        print(f"✗ 转换失败: {result.stderr}")
        return False

async def main():
    # 确保输出目录存在
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print("="*60)
    print("生成东北话早安问候音频 (Xiaobei音色)")
    print("="*60 + "\n")
    
    # 生成并转换音频文件
    for i, text in enumerate(TEXTS, 1):
        mp3_path = os.path.join(OUTPUT_DIR, f"morning_{i:02d}_xiaobei.mp3")
        pcm_path = os.path.join(OUTPUT_DIR, f"morning_{i:02d}_xiaobei.pcm")
        
        # 生成MP3
        await generate_mp3(text, mp3_path)
        
        # 转换PCM
        if os.path.exists(FFMPEG_PATH):
            convert_mp3_to_pcm(mp3_path, pcm_path)
        else:
            print(f"⚠️  ffmpeg未找到，请手动转换: {mp3_path}")
        
        print()
    
    print("="*60)
    print("✅ 全部完成！")
    print(f"📁 输出目录: {OUTPUT_DIR}")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(main())
```

### 使用方法

```powershell
# 运行脚本
cd D:\Unitree-R1-迎宾
C:\Users\Zirui\miniconda3\python.exe generate_northeast_xiaobei.py
```

---

## 常用命令

### 1. 批量转换现有MP3到PCM

```powershell
$ffmpeg = "D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffmpeg.exe"
$dir = "D:\Unitree-R1-迎宾\voice_pack\audio_morning"

Get-ChildItem "$dir\*.mp3" | ForEach-Object {
    $mp3 = $_.FullName
    $pcm = $mp3 -replace '\.mp3$', '.pcm'
    & $ffmpeg -i $mp3 -f s16le -acodec pcm_s16le -ac 1 -ar 16000 $pcm -y
    Write-Host "✓ 转换: $(Split-Path $pcm -Leaf)"
}
```

### 2. 检查PCM文件参数

```powershell
# 使用ffprobe检查PCM文件
$ffprobe = "D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffprobe.exe"
& $ffprobe -f s16le -ac 1 -ar 16000 -i "morning_01_xiaobei.pcm"
```

### 3. 播放PCM文件 (测试用)

```powershell
# 使用ffplay播放PCM
$ffplay = "D:\Unitree-R1-迎宾\ffmpeg_temp\ffmpeg-8.1.1-essentials_build\bin\ffplay.exe"
& $ffplay -f s16le -ac 1 -ar 16000 "morning_01_xiaobei.pcm"
```

---

## 故障排查

### 问题1: edge-tts 报错 "No module named 'edge_tts'"

**解决方案：**
```powershell
C:\Users\Zirui\miniconda3\python.exe -m pip install edge-tts
```

### 问题2: ffmpeg 报错 "File not found"

**原因：** ffmpeg 路径错误

**解决方案：**
1. 检查 ffmpeg.exe 是否存在
2. 更新脚本中的 `FFMPEG_PATH` 变量
3. 或者将 ffmpeg 添加到系统 PATH

### 问题3: 生成的PCM文件播放速度异常

**原因：** PCM 参数不匹配

**解决方案：**
- 确认 Unitree R1 的期望参数：`S16_LE, 16000Hz, mono`
- 检查 ffmpeg 命令中的参数是否正确
- 使用 `ffprobe` 检查生成的PCM文件

### 问题4: 音频内容不完整

**原因：** 文本过长，被截断

**解决方案：**
- 将长文本拆分成多个短句
- 每个音频文件控制在 5-10 秒
- 使用标点符号控制停顿

---

## 最佳实践

### ✅ DO (推荐做法)

1. **使用东北话音色** - `zh-CN-liaoning-XiaobeiNeural` 音质好，适合迎宾场景
2. **控制文本长度** - 每个音频 5-10 秒，避免过长
3. **保留MP3原文件** - 方便后期修改和重新生成
4. **使用版本命名** - 如 `morning_01_xiaobei_v2.pcm`
5. **测试后再部署** - 先用 `ffplay` 试听，再上传到机器人

### ❌ DON'T (避免做法)

1. **不要删除 ffmpeg 文件夹** - 已配置好，可重复使用
2. **不要手动修改PCM文件** - 容易损坏
3. **不要混用采样率** - 保持 16000Hz 统一
4. **不要忘记修改 show.cpp** - 只替换PCM文件不够，还需更新代码

---

## 附录：show.cpp 修改模板

```cpp
static const Scene SCENES[] = {
    // ... 其他场景 ...
    {
        .name = "audio_morning",
        .text = "早啊！今儿个又是元气满满的一天，祝您工作顺心，啥事儿都顺顺利利的！|"
                "清早儿好啊！哎妈呀，今儿个心情不错吧？带着好心情开启充实的一天！|"
                "早啊！迎着晨光来上班啦？今儿个工作可得劲儿了，万事如意！|"
                "早安呐～新的一天新的开始，给您加满油！工作轻松，啥都顺！",
        .audio_folder = "/home/unitree/voice_pack/audio_morning",
        .audio_files = {
            "morning_01_xiaobei.pcm",
            "morning_02_xiaobei.pcm",
            "morning_03_xiaobei.pcm",
            "morning_04_xiaobei.pcm"
        },
        .num_audio_files = 4
    },
    // ... 其他场景 ...
};
```

---

## 更新记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-05-18 | v1.0 | 初始版本，记录edge-tts + ffmpeg流程 |

---

**文档结束**

如有问题，请联系项目维护者：Zirui (西电杭研院 姜文教授团队)
