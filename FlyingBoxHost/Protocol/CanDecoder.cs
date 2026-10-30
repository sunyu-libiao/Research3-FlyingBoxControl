using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace FlyingBoxHost.Protocol;

/// <summary>一帧的解析结果：分类名 + 人能读的内容。</summary>
internal readonly record struct Decoded(string Kind, string Text);

/// <summary>
/// 按本工程固件的帧定义解释总线数据。
/// 依据 doc/电机CAN控制流程.md 与 doc/tof8通讯协议.docx，多字节字段均为小端。
/// </summary>
internal static class CanDecoder
{
    private const uint IdErrorMin = 0x0100;
    private const uint IdErrorMax = 0x017F;
    private const uint IdBindSn = 0x0313;
    private const uint IdParam = 0x0316;
    private const uint IdHeartbeat = 0x0380;
    private const uint IdRequest = 0x0408;
    private const uint IdResponse = 0x0409;
    private const uint IdEnable = 0x0413;
    private const uint IdAccelMain = 0x0414;
    private const uint IdSpeedMain = 0x0415;
    private const uint IdPositionMain = 0x0416;
    private const uint IdAccelSub = 0x0417;
    private const uint IdSpeedSub = 0x0418;
    private const uint IdPositionSub = 0x0419;
    private const uint IdParamAck = 0x041A;
    private const uint IdRemain = 0x0420;
    private const uint IdTorque = 0x0440;
    private const uint IdArm = 0x0700;
    private const uint IdTofReport = 0x0793;
    private const uint IdTofReportAlt = 0x0794;
    private const uint IdSpan = 0x10;

    /* 设备号段：电机 7~11，TOF 25~26（见驱动里的 MOTOR_DEV_BASE / TOF_DEV_BASE） */
    private const int TofDevBase = 25;

    public static Decoded Decode(uint id, byte[] data)
    {
        if ((id >= IdErrorMin) && (id <= IdErrorMax))
        {
            return new Decoded("错误/受理", DescribeError(id));
        }

        if (id == IdBindSn)
        {
            return new Decoded("电机-绑定", $"SN {Hex(data, 0, Math.Min(7, data.Length))}，设备号 {ByteAt(data, 7)}");
        }

        if (id == IdParam)
        {
            return new Decoded("电机-参数", $"{Hex(data, 0, Math.Min(7, data.Length))}，设备号 {ByteAt(data, 7)}");
        }

        if ((id >= IdHeartbeat) && (id < IdHeartbeat + IdSpan))
        {
            return new Decoded("电机-心跳", $"设备号 {id - IdHeartbeat}");
        }

        if (id == IdRequest)
        {
            byte dev = ByteAt(data, 0);
            byte sub = ByteAt(data, 1);
            string text = $"设备 {dev}，子码 {SubName(sub)}";
            return new Decoded(IsTofDev(dev) ? "TOF-请求" : "电机-查询", text);
        }

        if (id == IdResponse)
        {
            return DescribeResponse(data);
        }

        if (id == IdEnable)
        {
            byte command = ByteAt(data, 0);
            string mode = command switch
            {
                0x5A => "使能",
                0x4A => "释放",
                _ => $"0x{command:X2}",
            };
            return new Decoded("电机-使能", $"{mode}，马达号 {ByteAt(data, 4)}");
        }

        if ((id == IdAccelMain) || (id == IdSpeedMain) || (id == IdAccelSub) || (id == IdSpeedSub))
        {
            string what = (id == IdAccelMain) || (id == IdAccelSub) ? "加速度" : "速度";
            string channel = (id == IdAccelMain) || (id == IdSpeedMain) ? "主" : "从";
            return new Decoded("电机-运动参数",
                $"{channel}通道{what} {U32(data, 0)}，马达号 {ByteAt(data, 4)}");
        }

        if ((id == IdPositionMain) || (id == IdPositionSub))
        {
            string type = id == IdPositionMain ? TypeName(ByteAt(data, 5)) : "从通道";
            return new Decoded("电机-目标位置",
                $"{type} {S32(data, 0)}，马达号 {ByteAt(data, 4)}");
        }

        if (id == IdParamAck)
        {
            return new Decoded("电机-参数应答",
                $"被应答帧 0x{U16(data, 0):X4}，马达号 {ByteAt(data, 2)}");
        }

        if ((id >= IdRemain) && (id < IdRemain + IdSpan))
        {
            if (data.Length < 8)
            {
                return new Decoded("电机-剩余位移", $"长度不足：{Hex(data, 0, data.Length)}");
            }

            return new Decoded("电机-剩余位移",
                $"马达 {id - IdRemain}，主 {S32(data, 0)}，从 {S32(data, 4)}（0.1 mm）");
        }

        if ((id >= IdTorque) && (id < IdTorque + IdSpan))
        {
            return new Decoded("电机-力矩",
                $"马达 {id - IdTorque}，原始 {S16(data, 2)}");
        }

        if (id == IdArm)
        {
            return new Decoded("机械臂", $"{Hex(data, 0, data.Length)}");
        }

        if ((id == IdTofReport) || (id == IdTofReportAlt))
        {
            if (data.Length < 3)
            {
                return new Decoded("TOF-上报", $"长度不足：{Hex(data, 0, data.Length)}");
            }

            return new Decoded("TOF-上报",
                $"设备 {ByteAt(data, 0)}，距离 {U16(data, 1)}（0.1 mm）");
        }

        return new Decoded("未知", Hex(data, 0, data.Length));
    }

    private static Decoded DescribeResponse(byte[] data)
    {
        if (data.Length < 2)
        {
            return new Decoded("TOF-应答", $"长度不足：{Hex(data, 0, data.Length)}");
        }

        byte dev = data[0];
        byte sub = data[1];
        bool isTof = IsTofDev(dev);
        string kind = isTof ? "TOF-应答" : "电机-查询应答";

        switch (sub)
        {
            case 0x08:
                return new Decoded(kind, $"设备 {dev}，位置 {S32(data, 2)}（0.1 mm）");
            case 0x0C:
                return new Decoded(kind,
                    $"设备 {dev}，距离 {U16(data, 2)}（0.1 mm），序号 {ByteAt(data, 4)}");
            case 0x06:
                return new Decoded(kind, $"设备 {dev}，温度 {S16(data, 2)}（0.1 °C）");
            case 0x25:
                return new Decoded(kind, $"设备 {dev}，状态位 0x{ByteAt(data, 2):X2}");
            case 0x24:
                return new Decoded(kind, $"设备 {dev}，轮径写入应答");
            case 0x15:
                return new Decoded(kind, $"设备 {dev}，分辨率 {ByteAt(data, 2)}");
            case 0x10:
                return new Decoded(kind, $"设备 {dev}，频率 {ByteAt(data, 2)}");
            case 0xFF:
                return new Decoded(kind, $"设备 {dev}，版本 {Hex(data, 2, Math.Max(0, data.Length - 2))}");
            default:
                return new Decoded(kind,
                    $"设备 {dev}，子码 0x{sub:X2}：{Hex(data, 2, Math.Max(0, data.Length - 2))}");
        }
    }

    private static string DescribeError(uint id)
    {
        string name = id switch
        {
            0x0100 => "受理标记",
            0x0102 => "未到位",
            0x0104 => "堵转",
            0x0105 => "超温",
            0x0107 => "欠压",
            0x0108 => "断线",
            _ => "未知错误",
        };

        return $"0x{id:X4} {name}";
    }

    private static string SubName(byte sub)
    {
        return sub switch
        {
            0x06 => "0x06 温度",
            0x08 => "0x08 位置",
            0x24 => "0x24 写轮径",
            0x25 => "0x25 状态",
            0x0A => "0x0A 连续测距",
            0x0B => "0x0B 单次测距",
            0x0C => "0x0C 读数",
            0x15 => "0x15 分辨率",
            0x10 => "0x10 频率",
            0xFF => "0xFF 版本",
            _ => $"0x{sub:X2}",
        };
    }

    private static string TypeName(byte type)
    {
        return type switch
        {
            0x00 => "相对",
            0x05 => "绝对",
            0x0A => "归零",
            0x0B => "置零",
            _ => $"类型 0x{type:X2}",
        };
    }

    private static bool IsTofDev(byte dev)
    {
        return dev >= TofDevBase;
    }

    private static byte ByteAt(byte[] data, int index)
    {
        return index < data.Length ? data[index] : (byte)0;
    }

    private static int U16(byte[] data, int offset)
    {
        if (offset + 1 >= data.Length)
        {
            return 0;
        }

        return data[offset] | (data[offset + 1] << 8);
    }

    private static int S16(byte[] data, int offset)
    {
        return (short)U16(data, offset);
    }

    private static int S32(byte[] data, int offset)
    {
        if (offset + 3 >= data.Length)
        {
            return 0;
        }

        return data[offset]
             | (data[offset + 1] << 8)
             | (data[offset + 2] << 16)
             | (data[offset + 3] << 24);
    }

    private static long U32(byte[] data, int offset)
    {
        return (uint)S32(data, offset);
    }

    public static string Hex(byte[] data, int offset, int count)
    {
        if (count <= 0)
        {
            return "";
        }

        StringBuilder builder = new(count * 3);
        for (int i = 0; i < count; i++)
        {
            int index = offset + i;
            if (index >= data.Length)
            {
                break;
            }

            if (i > 0)
            {
                builder.Append(' ');
            }

            builder.Append(data[index].ToString("X2", CultureInfo.InvariantCulture));
        }

        return builder.ToString();
    }
}

/// <summary>一条待发的命令：帧 ID、载荷、给人看的名字。</summary>
internal readonly record struct OutFrame(uint Id, byte[] Data, string Note);

/// <summary>
/// 按固件 motor.c 的发送函数拼装请求帧，供上位机主动下发。
/// 与 CanDecoder 共用同一套帧定义：一个解、一个拼。
///
/// ★ 这里的默认参数（轮径 / 速度 / 加速度 / 方向位）抄自固件
///   s_defaultMotors 与 MOTOR_PARAMS_DEFAULT。固件改了而这里没跟着改时，
///   本面板下发的内容就会与固件不一致——方向位不一致会让电机朝反方向走。
/// </summary>
internal static class CanCommand
{
    /* 帧 ID，与固件 motor.c 的 MOTOR_ID_* 一致 */
    private const uint IdParam = 0x0316;
    private const uint IdMulti = 0x0408;
    private const uint IdEnable = 0x0413;
    private const uint IdAccelMain = 0x0414;
    private const uint IdSpeedMain = 0x0415;
    private const uint IdPositionMain = 0x0416;

    /* 子功能与命令字，与固件 motor.c 的宏一致 */
    private const byte SubWheel = 0x24;
    private const byte CmdEnable = 0x5A;
    private const byte PosRelative = 0x00;

    /* 0x0316 前 6 字节取自固件 MOTOR_PARAMS_DEFAULT，本面板不改这些参数，
       原样回填以保证“初始化”不会把固件调好的参数冲掉 */
    private const byte Param0 = 0xB4;
    private const byte Param1 = 0xBF;
    private const byte Param2 = 0xCF;
    private const byte Param3 = 0xAA;
    private const byte Param4 = 0x00;
    private const byte Param5 = 0x5F;

    /* 固件 s_defaultMotors 里五台的运动参数。逐台查表而不是共用一个常量：
       7/8/9 的轮径都是按实测行程反算的，7/8 控制皮带、9 控制直线机构，
       三者的“每转行程”与 10/11 差着量级；速度与加速度也必须与轮径同量级。
       若统一按一个默认值下发，本面板的“初始化”会把固件配好的参数冲掉，
       之后所有位移与速度又变成错误的比例。
       ★ 固件改 s_defaultMotors 时必须同步这里，否则两边不一致。 */
    private const ushort UncalWheel001mm = 60000;
    private const uint UncalSpeed001mms = 160000;
    private const uint UncalAccel001mms2 = 80000;

    private const byte MotorIdBase = 7;

    /// <summary>一台电机在固件里的轮径、速度、加速度，以及方向反转位。</summary>
    internal readonly record struct MotorDefaults(ushort Wheel001mm, uint Speed001mms,
                                                 uint Accel001mms2, bool Reverse);

    /// <summary>
    /// 按马达号取固件里的运动参数。键是马达号，不是界面下标。
    /// 固件改了 s_defaultMotors 而未同步这里时，本面板下发的内容会与固件不一致：
    /// 方向位不一致会让电机朝反方向走，轮径不一致会让所有位移整体缩放。
    /// </summary>
    public static MotorDefaults DefaultsOf(byte motorId)
    {
        return motorId switch
        {
            /* 下标 0：7 号，皮带，轮径按实测 1000mm→53mm 反算 */
            7 => new MotorDefaults(3180, 1600, 800, false),
            /* 下标 1：8 号，另一根皮带，同一套传动；安装方向相反 */
            8 => new MotorDefaults(3180, 1600, 800, true),
            /* 下标 2：9 号，直线机构，轮径按实测 55731.2mm→578mm 反算；反向 */
            9 => new MotorDefaults(622, 1600, 800, true),
            /* 下标 4：11 号，钩爪机构，未标定；反向 */
            11 => new MotorDefaults(UncalWheel001mm, UncalSpeed001mms, UncalAccel001mms2, true),
            /* 下标 3：10 号，未标定 */
            _ => new MotorDefaults(UncalWheel001mm, UncalSpeed001mms, UncalAccel001mms2, false),
        };
    }

    /// <summary>该马达号在固件里配置的轮径，0.01 mm。</summary>
    public static ushort DefaultWheel001mmOf(byte motorId)
    {
        return DefaultsOf(motorId).Wheel001mm;
    }

    /// <summary>
    /// 该马达号在固件里的方向反转位。界面据此预置复选框：
    /// 显示出来比藏在代码里安全，方向位弄错电机会朝反方向走。
    /// </summary>
    public static bool DefaultReverse(byte motorId)
    {
        return DefaultsOf(motorId).Reverse;
    }

    /// <summary>把一个 0~4 的下标换算成马达号；默认寻址下设备号与马达号相同。</summary>
    public static byte MotorIdOf(int index)
    {
        return (byte)(MotorIdBase + index);
    }

    /// <summary>
    /// 拼出“初始化 + 使能 + 相对位移”的完整序列，顺序与固件
    /// configOneMotor() + Motor_SetEnabled() + Motor_SetTargetPositionMain() 一致：
    /// 参数 → 轮径（协议要求连写三次）→ 加速度 → 速度 → 使能 → 位移。
    ///
    /// 轮径必须在下发运动命令之前写好，实测不配轮径运动指令不会执行；
    /// 速度与加速度只在本条运动命令之前生效，所以它们必须排在位移之前。
    /// </summary>
    /// <param name="motorId">马达号，同时用作默认寻址下的设备号</param>
    /// <param name="pos001mm">相对位移，0.1 mm，符号决定方向</param>
    /// <param name="reverse">0x0316 的方向反转位</param>
    public static List<OutFrame> InitAndMove(byte motorId, int pos001mm, bool reverse)
    {
        MotorDefaults defaults = DefaultsOf(motorId);
        byte[] wheel = Wheel(motorId, defaults.Wheel001mm);

        return new List<OutFrame>
        {
            new(IdParam, Param(motorId, reverse), "参数"),
            new(IdMulti, wheel, "轮径 1/3"),
            new(IdMulti, (byte[])wheel.Clone(), "轮径 2/3"),
            new(IdMulti, (byte[])wheel.Clone(), "轮径 3/3"),
            new(IdAccelMain, U32WithMotorId(defaults.Accel001mms2, motorId), "加速度"),
            new(IdSpeedMain, U32WithMotorId(defaults.Speed001mms, motorId), "速度"),
            new(IdEnable, Enable(motorId), "使能"),
            new(IdPositionMain, MoveRelative(pos001mm, motorId), "相对位移"),
        };
    }

    /// <summary>0x0316：整组参数 + 马达号 + 设备号。本面板只改方向位，其余原样回填。</summary>
    private static byte[] Param(byte motorId, bool reverse)
    {
        return new[]
        {
            Param0, Param1, Param2, Param3, Param4, Param5,
            /* bit5/bit4 保持 0：这两个位会关掉周期反馈，留着 0 才有 0x0420/0x0440 */
            (byte)((reverse ? 0x80 : 0x00) | (motorId & 0x0F)),
            motorId,
        };
    }

    /// <summary>0x0408 子功能 0x24：轮径，设备号 + 子功能 + u16 LE（0.01 mm）。</summary>
    private static byte[] Wheel(byte devId, ushort wheel001mm)
    {
        return new[]
        {
            devId,
            SubWheel,
            (byte)(wheel001mm & 0xFF),
            (byte)(wheel001mm >> 8),
        };
    }

    /// <summary>0x0413：使能。第 5 字节是马达号，不是设备号。</summary>
    private static byte[] Enable(byte motorId)
    {
        return new[] { CmdEnable, (byte)0x00, (byte)0x00, (byte)0x00, motorId };
    }

    /// <summary>0x0416：目标位置 s32 LE + 马达号 + 目标类型。</summary>
    private static byte[] MoveRelative(int pos001mm, byte motorId)
    {
        uint raw = unchecked((uint)pos001mm);

        return new[]
        {
            (byte)(raw & 0xFF),
            (byte)((raw >> 8) & 0xFF),
            (byte)((raw >> 16) & 0xFF),
            (byte)((raw >> 24) & 0xFF),
            motorId,
            PosRelative,
        };
    }

    /// <summary>0x0414 / 0x0415 共用：u32 LE + 马达号。</summary>
    private static byte[] U32WithMotorId(uint value, byte motorId)
    {
        return new[]
        {
            (byte)(value & 0xFF),
            (byte)((value >> 8) & 0xFF),
            (byte)((value >> 16) & 0xFF),
            (byte)((value >> 24) & 0xFF),
            motorId,
        };
    }
}
