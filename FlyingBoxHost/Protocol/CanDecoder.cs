using System;
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
