using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace FlyingBoxHost.UsbCan;

/// <summary>从总线抓到的一帧。时间戳取本机单调时钟。</summary>
internal readonly record struct RawFrame(long TickMs, uint Id, byte[] Data);

/// <summary>
/// CANable2（normaldotcom 固件）的 SLCAN 通道。设备在系统里是 CDC 串口，
/// 用 LAWICEL 文本命令控制，不需要厂商 DLL。
///
/// 实测这套固件的行为（决定了下面哪些做法是对的）：
///   V   回版本串（git describe）
///   S   静默成功
///   O   打开通道，之后持续推帧 —— **唯一可靠的打开方式**
///   C   关闭通道，状态会跨进程保留（下一次打开必须重新 O）
///   F/N/Z/l(回环) 未实现，静默
/// 因此打开时一律发 O；且通道可能因上一次会话的 C 处于关闭状态，
/// 所以发完 O 要确认真的有字节进来，没进来就重发。
///
/// 只做被动接收：除设置波特率、打开通道以及主动下发的控制帧外，不发其它数据帧。
/// </summary>
internal sealed class SlcanDevice : IDisposable
{
    /* lawicel 波特率码：0=10k 1=20k 2=50k 3=100k 4=125k 5=250k 6=500k 7=800k 8=1M */
    public const byte Code1Mbps = 8;

    private const int SerialBitrate = 115200;   /* CDC 下无实际意义，仅为满足 SerialPort */
    private const int SettleMs = 200;
    private const int ProbeMs = 300;
    private const int OpenAttempts = 5;
    private const char Bell = '\a';
    private const int MaxLineLength = 96;

    private readonly ConcurrentQueue<RawFrame> frames = new();
    private readonly Stopwatch clock = Stopwatch.StartNew();
    private readonly StringBuilder lineBuffer = new(128);
    private readonly object portLock = new();

    private SerialPort? port;
    private Thread? rxThread;
    private volatile bool running;
    private long dropped;
    private long rawBytes;
    private long lineCount;
    private long malformedLines;
    private long lastByteTickMs;

    public bool IsOpen { get; private set; }

    public string Version { get; private set; } = "";

    /// <summary>接收线程退出的原因；非空表示链路已断。</summary>
    public string Fault { get; private set; } = "";

    public long DroppedFrames => Interlocked.Read(ref dropped);

    /// <summary>串口层收到的原始字节数。为 0 说明通道没打开或总线没流量。</summary>
    public long RawBytes => Interlocked.Read(ref rawBytes);

    public long LineCount => Interlocked.Read(ref lineCount);

    /// <summary>无法识别的行数。非零而帧数为 0 说明帧格式判断有误。</summary>
    public long MalformedLines => Interlocked.Read(ref malformedLines);

    public int PendingCount => frames.Count;

    /// <summary>距最后一次收到字节的毫秒数，用于判定通道是否已经哑掉。</summary>
    public long IdleMs => clock.ElapsedMilliseconds - Interlocked.Read(ref lastByteTickMs);

    public static string[] GetPortNames()
    {
        return SerialPort.GetPortNames();
    }

    /// <summary>
    /// 打开串口并打开 CAN 通道。该固件只有 O 能可靠打开通道，且通道状态
    /// 会跨会话保留（上一次若发过 C，这次必须重新 O），所以发完 O 会确认
    /// 是否真有字节进来，没有就重发。
    /// </summary>
    /// <param name="portName">COM 口名</param>
    /// <param name="bitrateCode">lawicel 波特率码</param>
    /// <param name="message">结果说明</param>
    /// <returns>通道建立且确认有数据流动返回 true</returns>
    public bool Open(string portName, byte bitrateCode, out string message)
    {
        if (IsOpen)
        {
            message = "已打开";
            return true;
        }

        try
        {
            port = new SerialPort(portName, SerialBitrate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 100,
                WriteTimeout = 1000,
                DtrEnable = true,
                RtsEnable = true,
            };
            port.Open();
            port.DiscardInBuffer();
            port.DiscardOutBuffer();
        }
        catch (Exception ex)
        {
            message = "打开 " + portName + " 失败：" + ex.Message;
            DisposePort();
            return false;
        }

        Thread.Sleep(SettleMs);

        Version = QueryVersion();

        if (!SendSilent("S" + bitrateCode.ToString(CultureInfo.InvariantCulture)))
        {
            message = "设置波特率码 S" + bitrateCode + " 被设备拒绝（收到 BEL）";
            DisposePort();
            return false;
        }

        /* 接收线程要在打开通道之前跑起来，否则确认阶段收到的字节会被丢掉 */
        IsOpen = true;
        Fault = "";
        running = true;
        Interlocked.Exchange(ref lastByteTickMs, clock.ElapsedMilliseconds);
        rxThread = new Thread(RxLoop)
        {
            IsBackground = true,
            Name = "slcan-rx",
        };
        rxThread.Start();

        long totalBytes = 0;
        for (int attempt = 1; attempt <= OpenAttempts; attempt++)
        {
            if (!SendSilent("O"))
            {
                message = "打开通道 O 被设备拒绝（收到 BEL）";
                Close();
                return false;
            }

            Thread.Sleep(ProbeMs);
            totalBytes = RawBytes;
            if (totalBytes > 0)
            {
                break;
            }
        }

        ResetCounters();
        Interlocked.Exchange(ref lastByteTickMs, clock.ElapsedMilliseconds);

        if (totalBytes == 0)
        {
            message = portName + " 串口已打开，但发 " + OpenAttempts + " 次 O 后仍无任何字节。" +
                      Environment.NewLine +
                      "可能原因：总线无流量 / 波特率不匹配 / CAN_H·CAN_L 未接好。" +
                      Environment.NewLine + "固件：" + (Version.Length > 0 ? Version : "未报告");
            return true;    /* 端口已开，交给界面显示无流量告警 */
        }

        message = portName + " 已连接，通道已打开" +
                  Environment.NewLine + "固件：" + (Version.Length > 0 ? Version : "未报告");
        return true;
    }

    /// <summary>
    /// 通道哑掉时重新发 O。界面的看门狗定时调用；返回 true 表示通道已有数据。
    /// </summary>
    public bool Kick()
    {
        if (!IsOpen)
        {
            return false;
        }

        if (!SendSilent("O"))
        {
            Fault = "重发 O 被设备拒绝";
            return false;
        }

        Thread.Sleep(ProbeMs);
        return RawBytes > 0;
    }

    private string QueryVersion()
    {
        if (!WriteRaw("V\r"))
        {
            return "";
        }

        StringBuilder reply = new();
        DateTime deadline = DateTime.UtcNow.AddMilliseconds(ProbeMs);
        while (DateTime.UtcNow < deadline)
        {
            string chunk = ReadChunk();
            if (chunk.Length > 0)
            {
                reply.Append(chunk);
                if (chunk.IndexOf('\r') >= 0)
                {
                    break;
                }
            }

            Thread.Sleep(5);
        }

        return reply.ToString().Replace("\r", "").Replace("\n", "").Trim();
    }

    /// <summary>
    /// 发一条不需要回应的 SLCAN 命令。该固件成功时不回任何字节，
    /// 只有收到 BEL 才算拒绝；把“没回应”当失败会永远开不了通道。
    /// </summary>
    private bool SendSilent(string command)
    {
        if (!WriteRaw(command + "\r"))
        {
            return false;
        }

        DateTime deadline = DateTime.UtcNow.AddMilliseconds(120);
        while (DateTime.UtcNow < deadline)
        {
            if (ReadChunk().IndexOf(Bell) >= 0)
            {
                return false;
            }

            Thread.Sleep(5);
        }

        return true;
    }

    /// <summary>
    /// 发一帧标准数据帧（11 位标识符）。SLCAN 文本格式为
    /// t 加 3 位十六进制标识符、1 位十六进制长度、随后每字节 2 位十六进制。
    ///
    /// 这里只写、不读：读端口留给接收线程，否则会把帧从接收流里吃掉。
    /// 返回 wire 为实际写出的文本，便于在界面上核对字节。
    /// </summary>
    /// <param name="id">11 位标准标识符</param>
    /// <param name="data">载荷，最多 8 字节</param>
    /// <param name="wire">实际写出的 SLCAN 文本</param>
    /// <returns>写成功返回 true</returns>
    public bool SendFrame(uint id, byte[] data, out string wire)
    {
        wire = "";

        if (!IsOpen)
        {
            wire = "设备未打开";
            return false;
        }

        if (id > 0x7FFu)
        {
            wire = "标识符超出 11 位范围";
            return false;
        }

        if ((data.Length < 1) || (data.Length > 8))
        {
            wire = "载荷长度必须在 1~8 之间，当前 " + data.Length;
            return false;
        }

        StringBuilder line = new("t");
        line.Append(id.ToString("X3", CultureInfo.InvariantCulture));
        line.Append(data.Length.ToString("X1", CultureInfo.InvariantCulture));
        foreach (byte b in data)
        {
            line.Append(b.ToString("X2", CultureInfo.InvariantCulture));
        }

        line.Append('\r');
        wire = line.ToString().TrimEnd('\r');
        return WriteRaw(line.ToString());
    }

    private bool WriteRaw(string text)
    {
        lock (portLock)
        {
            if (port == null || !port.IsOpen)
            {
                return false;
            }

            try
            {
                port.Write(text);
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }
    }

    private string ReadChunk()
    {
        SerialPort? p = port;
        if (p == null || !p.IsOpen)
        {
            return "";
        }

        try
        {
            string text = p.ReadExisting();
            if (text.Length > 0)
            {
                Interlocked.Add(ref rawBytes, text.Length);
            }

            return text;
        }
        catch (Exception)
        {
            return "";
        }
    }

    private void RxLoop()
    {
        byte[] buffer = new byte[1024];

        while (running)
        {
            SerialPort? p = port;
            if (p == null || !p.IsOpen)
            {
                Fault = "串口已关闭";
                break;
            }

            int read;
            try
            {
                read = p.Read(buffer, 0, buffer.Length);
            }
            catch (TimeoutException)
            {
                continue;
            }
            catch (Exception ex)
            {
                Fault = "读取中断：" + ex.Message;
                break;
            }

            if (read <= 0)
            {
                continue;
            }

            Interlocked.Add(ref rawBytes, read);
            Interlocked.Exchange(ref lastByteTickMs, clock.ElapsedMilliseconds);

            for (int i = 0; i < read; i++)
            {
                char c = (char)buffer[i];
                if ((c == '\r') || (c == '\n'))
                {
                    if (lineBuffer.Length > 0)
                    {
                        HandleLine(lineBuffer.ToString());
                        lineBuffer.Clear();
                    }

                    continue;
                }

                if (lineBuffer.Length < MaxLineLength)
                {
                    lineBuffer.Append(c);
                }
                else
                {
                    lineBuffer.Clear();
                    Interlocked.Increment(ref malformedLines);
                }
            }
        }

        running = false;
    }

    /// <summary>
    /// 解析一行 SLCAN 接收报文：
    ///   t&lt;III&gt;&lt;L&gt;&lt;DD..&gt;   标准数据帧
    ///   T&lt;IIIIIIII&gt;&lt;L&gt;&lt;DD..&gt; 扩展数据帧
    ///   r / R 远程帧，本工程不用，丢弃
    /// </summary>
    private void HandleLine(string line)
    {
        Interlocked.Increment(ref lineCount);

        if (line.Length < 1)
        {
            return;
        }

        char type = line[0];
        if ((type != 't') && (type != 'T'))
        {
            if ((type != 'r') && (type != 'R'))
            {
                Interlocked.Increment(ref malformedLines);
            }

            return;
        }

        int idDigits = (type == 't') ? 3 : 8;
        int headerLen = 1 + idDigits + 1;
        if (line.Length < headerLen)
        {
            Interlocked.Increment(ref malformedLines);
            return;
        }

        if (!uint.TryParse(line.AsSpan(1, idDigits), NumberStyles.HexNumber,
                           CultureInfo.InvariantCulture, out uint id))
        {
            Interlocked.Increment(ref malformedLines);
            return;
        }

        if (!int.TryParse(line.AsSpan(1 + idDigits, 1), NumberStyles.HexNumber,
                          CultureInfo.InvariantCulture, out int dlc))
        {
            Interlocked.Increment(ref malformedLines);
            return;
        }

        if (dlc is < 0 or > 8)
        {
            Interlocked.Increment(ref malformedLines);
            return;
        }

        int available = (line.Length - headerLen) / 2;
        if (available < dlc)
        {
            Interlocked.Increment(ref malformedLines);
            return;
        }

        byte[] data = new byte[dlc];
        for (int i = 0; i < dlc; i++)
        {
            if (!byte.TryParse(line.AsSpan(headerLen + (i * 2), 2), NumberStyles.HexNumber,
                               CultureInfo.InvariantCulture, out byte value))
            {
                Interlocked.Increment(ref malformedLines);
                return;
            }

            data[i] = value;
        }

        frames.Enqueue(new RawFrame(clock.ElapsedMilliseconds, id, data));
    }

    public bool TryDequeue(out RawFrame frame)
    {
        return frames.TryDequeue(out frame);
    }

    public void DropQueue()
    {
        while (frames.TryDequeue(out _))
        {
        }
    }

    public void ResetCounters()
    {
        Interlocked.Exchange(ref rawBytes, 0);
        Interlocked.Exchange(ref lineCount, 0);
        Interlocked.Exchange(ref malformedLines, 0);
        Interlocked.Exchange(ref dropped, 0);
    }

    /// <summary>界面来不及消费时丢弃最旧的帧，只计数，防止内存无限增长。</summary>
    public void TrimQueue(int maxPending)
    {
        while (frames.Count > maxPending)
        {
            if (!frames.TryDequeue(out _))
            {
                return;
            }

            Interlocked.Increment(ref dropped);
        }
    }

    public void Close()
    {
        if (!IsOpen)
        {
            return;
        }

        running = false;

        /* 关通道。注意该固件会记住这个状态，下次打开必须重新发 O */
        WriteRaw("C\r");

        rxThread?.Join(TimeSpan.FromMilliseconds(500));
        rxThread = null;

        DisposePort();
        IsOpen = false;
        Version = "";
    }

    private void DisposePort()
    {
        lock (portLock)
        {
            if (port == null)
            {
                return;
            }

            try
            {
                if (port.IsOpen)
                {
                    port.Close();
                }
            }
            catch (Exception)
            {
                /* 设备已拔出时关闭会抛，无需处理 */
            }

            port.Dispose();
            port = null;
        }
    }

    public void Dispose()
    {
        Close();
    }
}
