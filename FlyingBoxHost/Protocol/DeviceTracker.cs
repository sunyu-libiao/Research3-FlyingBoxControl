using System;
using FlyingBoxHost.UsbCan;

namespace FlyingBoxHost.Protocol;

/// <summary>
/// 把总线上抓到的帧按设备归类，维护 5 台电机与 2 个激光测距模块的实时状态。
/// 归类规则与固件里的推进逻辑保持一致，均依据 doc/电机CAN控制流程.md。
/// </summary>
internal sealed class DeviceTracker
{
    public const int MotorCount = 5;
    public const int TofCount = 2;

    private const int MotorDevBase = 7;
    private const int TofDevBase = 25;

    private const uint IdHeartbeat = 0x0380;
    private const uint IdResponse = 0x0409;
    private const uint IdAck = 0x041A;
    private const uint IdRemain = 0x0420;
    private const uint IdTorque = 0x0440;
    private const uint IdTofReport = 0x0793;
    private const uint IdErrorMin = 0x0100;
    private const uint IdErrorMax = 0x017F;
    private const uint IdSpan = 0x10;

    /* 心跳的设备号窗口必须覆盖到 TOF（设备号 25/26），不能用 0x10 那个
       只适合电机 7~11 的窄窗口，否则 0x399/0x39A 会被整段漏掉。 */
    private const uint IdHeartbeatSpan = 0x80;

    /* 距离原始值的无效判据，与固件 tof.h 一致 */
    private const int TofRawMinValid = 10;
    private const int TofRawInvalidLong = 9999;
    private const int TofRawInvalidShort = 8888;

    private readonly MotorView[] motors = new MotorView[MotorCount];
    private readonly TofView[] tofs = new TofView[TofCount];

    public DeviceTracker()
    {
        for (int i = 0; i < MotorCount; i++)
        {
            motors[i] = new MotorView(MotorDevBase + i);
        }

        for (int i = 0; i < TofCount; i++)
        {
            tofs[i] = new TofView(TofDevBase + i);
        }
    }

    public MotorView[] Motors => motors;

    public TofView[] Tofs => tofs;

    public long TotalFrames { get; private set; }

    public long ErrorFrames { get; private set; }

    public uint LastErrorId { get; private set; }

    public long UnknownFrames { get; private set; }

    public long LastFrameTickMs { get; private set; }

    public bool HasAnyFrame => TotalFrames > 0;

    public void Apply(RawFrame frame)
    {
        TotalFrames++;
        LastFrameTickMs = frame.TickMs;

        uint id = frame.Id;

        if ((id >= IdErrorMin) && (id <= IdErrorMax))
        {
            /* 0x0100 是“运动指令受理”标记，不算错误 */
            if (id != IdErrorMin)
            {
                ErrorFrames++;
                LastErrorId = id;
            }

            return;
        }

        if ((id >= IdHeartbeat) && (id < IdHeartbeat + IdHeartbeatSpan))
        {
            int dev = (int)(id - IdHeartbeat);

            /* 心跳的设备号段既能是电机也能是 TOF，按设备号区分归属 */
            MotorView? motor = FindMotorByDev(dev);
            if (motor != null)
            {
                motor.Heartbeats++;
                Touch(motor, frame.TickMs);
            }
            else
            {
                TofView? tof = FindTofByDev(dev);
                if (tof != null)
                {
                    tof.Heartbeats++;
                    Touch(tof, frame.TickMs);
                }
            }

            return;
        }

        if ((id >= IdRemain) && (id < IdRemain + IdSpan))
        {
            MotorView? motor = FindMotorByDev((int)(id - IdRemain));
            if ((motor != null) && (frame.Data.Length >= 8))
            {
                motor.RemainMain = ReadS32(frame.Data, 0);
                motor.RemainSub = ReadS32(frame.Data, 4);
                Touch(motor, frame.TickMs);
            }

            return;
        }

        if ((id >= IdTorque) && (id < IdTorque + IdSpan))
        {
            MotorView? motor = FindMotorByDev((int)(id - IdTorque));
            if ((motor != null) && (frame.Data.Length >= 4))
            {
                motor.Torque = (short)(frame.Data[2] | (frame.Data[3] << 8));
                Touch(motor, frame.TickMs);
            }

            return;
        }

        if (id == IdAck)
        {
            if (frame.Data.Length < 3)
            {
                return;
            }

            MotorView? motor = FindMotorByDev(frame.Data[2]);
            if (motor != null)
            {
                motor.Acks++;
                Touch(motor, frame.TickMs);
            }

            return;
        }

        if (id == IdTofReport)
        {
            ApplyTofReport(frame);
            return;
        }

        if (id == IdResponse)
        {
            ApplyResponse(frame);
            return;
        }

        if ((id >= 0x0408) && (id <= 0x0419))
        {
            return;     /* 这些是 MCU 发出的命令，界面上不需要 */
        }

        UnknownFrames++;
    }

    private void ApplyTofReport(RawFrame frame)
    {
        if (frame.Data.Length < 3)
        {
            return;
        }

        TofView? tof = FindTofByDev(frame.Data[0]);
        if (tof == null)
        {
            return;
        }

        int raw = frame.Data[1] | (frame.Data[2] << 8);
        StoreTofDistance(tof, raw, frame.TickMs);
    }

    private void ApplyResponse(RawFrame frame)
    {
        if (frame.Data.Length < 2)
        {
            return;
        }

        int dev = frame.Data[0];
        int sub = frame.Data[1];

        TofView? tof = FindTofByDev(dev);
        if (tof != null)
        {
            switch (sub)
            {
                case 0x0C:      /* 读数：B2~B3 距离，B4 递增序号 */
                    if (frame.Data.Length >= 4)
                    {
                        int raw = frame.Data[2] | (frame.Data[3] << 8);
                        StoreTofDistance(tof, raw, frame.TickMs);
                    }

                    if (frame.Data.Length >= 5)
                    {
                        tof.Seq = frame.Data[4];
                    }

                    return;
                case 0x15:      /* 分辨率 */
                    if (frame.Data.Length >= 3)
                    {
                        tof.Resolution = frame.Data[2];
                    }

                    return;
                case 0x10:      /* 频率 */
                    if (frame.Data.Length >= 3)
                    {
                        tof.Frequency = frame.Data[2];
                    }

                    return;
            }

            return;
        }

        MotorView? motor = FindMotorByDev(dev);
        if (motor == null)
        {
            return;
        }

        switch (sub)
        {
            case 0x08:      /* 位置 */
                if (frame.Data.Length >= 6)
                {
                    motor.Position = ReadS32(frame.Data, 2);
                }

                break;
            case 0x06:      /* 温度 */
                if (frame.Data.Length >= 4)
                {
                    motor.Temperature = (short)(frame.Data[2] | (frame.Data[3] << 8));
                }

                break;
            case 0x25:      /* 状态位 */
                if (frame.Data.Length >= 3)
                {
                    motor.Status = frame.Data[2];
                }

                break;
            default:
                break;
        }

        Touch(motor, frame.TickMs);
    }

    private static void StoreTofDistance(TofView tof, int raw, long tickMs)
    {
        tof.RawDistance = raw;
        tof.Valid = (raw > TofRawMinValid) &&
                    (raw != TofRawInvalidLong) &&
                    (raw != TofRawInvalidShort);
        Touch(tof, tickMs);
    }

    private static void Touch(MotorView motor, long tickMs)
    {
        motor.Frames++;
        motor.LastSeenMs = tickMs;
        motor.HasData = true;
    }

    private static void Touch(TofView tof, long tickMs)
    {
        tof.Frames++;
        tof.LastSeenMs = tickMs;
        tof.HasData = true;
    }

    private MotorView? FindMotorByDev(int devId)
    {
        int index = devId - MotorDevBase;
        return (index >= 0 && index < MotorCount) ? motors[index] : null;
    }

    private TofView? FindTofByDev(int devId)
    {
        int index = devId - TofDevBase;
        return (index >= 0 && index < TofCount) ? tofs[index] : null;
    }

    private static long ReadS32(byte[] data, int offset)
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

    public void Reset()
    {
        TotalFrames = 0;
        ErrorFrames = 0;
        LastErrorId = 0;
        UnknownFrames = 0;
        LastFrameTickMs = 0;

        for (int i = 0; i < MotorCount; i++)
        {
            MotorView motor = motors[i];
            motor.RemainMain = 0;
            motor.RemainSub = 0;
            motor.Torque = 0;
            motor.Position = 0;
            motor.Status = 0;
            motor.Temperature = 0;
            motor.Heartbeats = 0;
            motor.Acks = 0;
            motor.Frames = 0;
            motor.LastSeenMs = 0;
            motor.HasData = false;
        }

        for (int i = 0; i < TofCount; i++)
        {
            TofView tof = tofs[i];
            tof.RawDistance = 0;
            tof.Valid = false;
            tof.Seq = 0;
            tof.Frequency = 0;
            tof.Resolution = 0;
            tof.Frames = 0;
            tof.Heartbeats = 0;
            tof.LastSeenMs = 0;
            tof.HasData = false;
        }
    }
}
