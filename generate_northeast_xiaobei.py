"""
生成东北话早安问候PCM音频文件
音色: zh-CN-liaoning-XiaobeiNeural (辽宁东北话女声)
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

async def generate_mp3(text, output_path, voice="zh-CN-liaoning-XiaobeiNeural"):
    """使用edge-tts生成MP3文件"""
    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(output_path)
    print(f"✓ 生成MP3: {output_path}")

async def convert_mp3_to_pcm(mp3_path, pcm_path, ffmpeg_path="ffmpeg"):
    """将MP3转换为PCM格式"""
    cmd = [
        ffmpeg_path,
        "-i", mp3_path,           # 输入文件
        "-f", "s16le",            # 输出格式: 16-bit signed little-endian PCM
        "-acodec", "pcm_s16le",   # 音频编解码器
        "-ac", "1",               # 单声道
        "-ar", "16000",           # 采样率 16000Hz
        pcm_path,                  # 输出文件
        "-y"                       # 覆盖已存在的文件
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ 转换PCM: {pcm_path}")
            return True
        else:
            print(f"✗ 转换失败: {result.stderr}")
            return False
    except Exception as e:
        print(f"✗ 转换出错: {e}")
        return False

async def main():
    # 确保输出目录存在
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 检查ffmpeg是否可用
    ffmpeg_available = False
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True)
        ffmpeg_available = True
        print("✓ ffmpeg已安装，可以直接生成PCM\n")
    except FileNotFoundError:
        print("✗ ffmpeg未安装，将只生成MP3文件")
        print("  请手动将MP3转换为PCM格式（16-bit, mono, 16000Hz）\n")

    # 生成音频文件
    for i, text in enumerate(TEXTS, 1):
        mp3_path = os.path.join(OUTPUT_DIR, f"morning_{i:02d}_xiaobei.mp3")
        pcm_path = os.path.join(OUTPUT_DIR, f"morning_{i:02d}_xiaobei.pcm")

        # 生成MP3
        await generate_mp3(text, mp3_path)

        # 转换PCM
        if ffmpeg_available:
            await convert_mp3_to_pcm(mp3_path, pcm_path)
        else:
            print(f"  等待手动转换: {mp3_path} -> {pcm_path}")

        print()

    print("="*60)
    if ffmpeg_available:
        print("✅ 全部完成！PCM文件已生成：")
    else:
        print("⚠️  MP3文件已生成，但需要手动转换：")
    print(f"   目录: {OUTPUT_DIR}")
    print("\nPCM转换参数: 16-bit signed PCM, mono, 16000Hz, raw PCM (无WAV头)")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(main())
