#pragma once

/*
 * R1 right arm teach points.
 *
 * This file stores the recorded right-arm joint positions used by the low-level
 * motion test programs. Keeping points here makes the motion code shorter and
 * makes future point updates easier.
 *
 * 你可以改：
 *   - 每个点位里的 5 个 float 数值。
 *   - 点位中文说明，例如“零件上方”“抓取位置”。
 *
 * 暂时不要改：
 *   - RightArmPose 的数组长度 5。
 *   - 数组顺序。
 *
 * 你应该学会：
 *   - 点位文件只负责保存“目标角度”。
 *   - 运动程序负责“怎么从当前角度走到目标角度”。
 *   - 这两件事分开，代码会更容易维护。
 *
 * 数组顺序固定为：
 *   [R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_SHOULDER_YAW, R_ELBOW, R_WRIST_ROLL]
 *   [右肩前后, 右肩左右, 右肩旋转, 右肘, 右腕旋转]
 */

#include <array>

using RightArmPose = std::array<float, 5>;

struct RightArmTeachPoint {
    const char* name;
    RightArmPose pose;
    const char* note;
};

inline constexpr RightArmTeachPoint kHome{
    "HOME / 安全初始位",
    {0.172741f, -0.120406f, 0.000409f, 0.838659f, -0.010067f},
    "安全初始位"
};

inline constexpr RightArmTeachPoint kPartAbove{
    "PART_ABOVE / 零件上方",
    {-0.746114f, -0.062204f, -0.316353f, 0.729907f, -0.009204f},
    "零件上方"
};

inline constexpr RightArmTeachPoint kPartGrasp{
    "PART_GRASP / 抓取位置",
    {-0.391189f, -0.060323f, -0.269895f, 0.507046f, -1.600719f},
    "抓取位置"
};

inline constexpr RightArmTeachPoint kPartLift{
    "PART_LIFT / 抓取抬高位",
    {-0.748439f, -0.104209f, -0.272862f, 0.480116f, -1.600780f},
    "抓取抬高位"
};

inline constexpr RightArmTeachPoint kAssemblyAbove{
    "ASSEMBLY_ABOVE / 装配位上方",
    {-0.747348f, 0.191436f, -0.020527f, 0.380311f, -1.600961f},
    "装配位上方"
};

inline constexpr RightArmTeachPoint kAssemblyInsert{
    "ASSEMBLY_INSERT / 装配插入位",
    {-0.489747f, 0.126368f, -0.022329f, 0.356968f, -1.600432f},
    "装配插入位"
};

inline constexpr RightArmTeachPoint kReturnHome{
    "RETURN_HOME / 返回安全位",
    {0.058477f, -0.103460f, -0.124041f, 1.186665f, 0.035393f},
    "返回安全位"
};

inline constexpr std::array<RightArmTeachPoint, 7> kRightArmTeachPoints{{
    kHome,
    kPartAbove,
    kPartGrasp,
    kPartLift,
    kAssemblyAbove,
    kAssemblyInsert,
    kReturnHome,
}};
