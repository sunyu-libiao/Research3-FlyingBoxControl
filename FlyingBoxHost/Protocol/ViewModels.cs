using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace FlyingBoxHost.Protocol;

/// <summary>属性变更通知的公共基类，供界面绑定。</summary>
public abstract class ObservableObject : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    protected void Set<T>(ref T field, T value, [CallerMemberName] string? name = null)
    {
        if (EqualityComparer<T>.Default.Equals(field, value))
        {
            return;
        }

        field = value;
        Raise(name);
    }

    protected void Raise(string? name)
    {
        if (name != null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }
}

/// <summary>一台电机的界面状态。</summary>
public sealed class MotorView : ObservableObject
{
    private long remainMain;
    private long remainSub;
    private int torque;
    private long position;
    private int status;
    private int temperature;
    private long heartbeats;
    private long acks;
    private long frames;
    private long lastSeenMs;
    private bool hasData;

    public MotorView(int devId)
    {
        DevId = devId;
    }

    public int DevId { get; }

    public string Title => "电机 " + DevId;

    public long RemainMain
    {
        get => remainMain;
        set
        {
            Set(ref remainMain, value);
            Raise(nameof(RemainMainText));
        }
    }

    public long RemainSub
    {
        get => remainSub;
        set
        {
            Set(ref remainSub, value);
            Raise(nameof(RemainSubText));
        }
    }

    public int Torque
    {
        get => torque;
        set => Set(ref torque, value);
    }

    public long Position
    {
        get => position;
        set
        {
            Set(ref position, value);
            Raise(nameof(PositionText));
        }
    }

    public int Status
    {
        get => status;
        set
        {
            Set(ref status, value);
            Raise(nameof(StatusText));
        }
    }

    public int Temperature
    {
        get => temperature;
        set
        {
            Set(ref temperature, value);
            Raise(nameof(TemperatureText));
        }
    }

    public long Heartbeats
    {
        get => heartbeats;
        set => Set(ref heartbeats, value);
    }

    public long Acks
    {
        get => acks;
        set => Set(ref acks, value);
    }

    public long Frames
    {
        get => frames;
        set => Set(ref frames, value);
    }

    public long LastSeenMs
    {
        get => lastSeenMs;
        set => Set(ref lastSeenMs, value);
    }

    public bool HasData
    {
        get => hasData;
        set => Set(ref hasData, value);
    }

    /* 0.1 mm 协议单位换成 mm 显示 */
    public string RemainMainText => RemainMain == 0 ? "0" : (RemainMain / 10.0).ToString("F1") + " mm";

    public string RemainSubText => RemainSub == 0 ? "0" : (RemainSub / 10.0).ToString("F1") + " mm";

    public string PositionText => (Position / 10.0).ToString("F1") + " mm";

    public string TemperatureText => (Temperature / 10.0).ToString("F1") + " °C";

    public string StatusText
    {
        get
        {
            List<string> parts = new();
            if ((Status & 0x01) != 0)
            {
                parts.Add("锁定");
            }

            if ((Status & 0x02) != 0)
            {
                parts.Add("已归零");
            }

            return parts.Count == 0 ? "0x" + Status.ToString("X2") : string.Join("/", parts);
        }
    }
}

/// <summary>一个激光测距模块的界面状态。</summary>
public sealed class TofView : ObservableObject
{
    private int rawDistance;
    private bool valid;
    private int seq;
    private int frequency;
    private int resolution;
    private long frames;
    private long heartbeats;
    private long lastSeenMs;
    private bool hasData;

    public TofView(int devId)
    {
        DevId = devId;
    }

    public int DevId { get; }

    public string Title => "TOF " + DevId;

    public int RawDistance
    {
        get => rawDistance;
        set
        {
            Set(ref rawDistance, value);
            Raise(nameof(DistanceText));
            Raise(nameof(RawText));
        }
    }

    public bool Valid
    {
        get => valid;
        set => Set(ref valid, value);
    }

    public int Seq
    {
        get => seq;
        set => Set(ref seq, value);
    }

    public int Frequency
    {
        get => frequency;
        set => Set(ref frequency, value);
    }

    public int Resolution
    {
        get => resolution;
        set => Set(ref resolution, value);
    }

    public long Frames
    {
        get => frames;
        set => Set(ref frames, value);
    }

    public long Heartbeats
    {
        get => heartbeats;
        set => Set(ref heartbeats, value);
    }

    public long LastSeenMs
    {
        get => lastSeenMs;
        set => Set(ref lastSeenMs, value);
    }

    public bool HasData
    {
        get => hasData;
        set => Set(ref hasData, value);
    }

    public string DistanceText => Valid ? (RawDistance / 10.0).ToString("F1") + " mm" : "无效";

    public string RawText => Valid
        ? "0x" + RawDistance.ToString("X4")
        : "0x" + RawDistance.ToString("X4") + "（哨兵值或过小）";
}
