using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Threading;
using FlyingBoxHost.Protocol;
using FlyingBoxHost.UsbCan;

namespace FlyingBoxHost
{
    /// <summary>表格里的一行。</summary>
    public sealed record FrameRow(long Seq, string Time, string Id, string Kind,
                                  string Data, string Text);

    /// <summary>统计面板的一行，每次刷新重建。</summary>
    public sealed class StatRow
    {
        public string Id { get; init; } = "";
        public string Kind { get; init; } = "";
        public string Count { get; init; } = "";
        public string Rate { get; init; } = "";
        public string LastLength { get; init; } = "";
    }

    public partial class MainWindow : Window
    {
        /* 原始报文的界面缓存上限，只保留最近若干条 */
        private const int MaxRows = 200;
        private const int TickMs = 100;
        private const int MaxDrainPerTick = 4000;
        private const int MaxPendingFrames = 8000;

        /* 连续这么久没收到任何字节，就认为通道哑了，自动重发 O */
        private const int RecoverIdleMs = 1500;

        private readonly SlcanDevice device = new();
        private readonly DeviceTracker tracker = new();
        private readonly List<FrameRow> rows = new();
        private readonly DispatcherTimer timer = new();

        private long sequence;
        private long framesAtLastRate;
        private double frameRate;
        private int renderedRowCount = -1;
        private int recoverCount;

        public MainWindow()
        {
            InitializeComponent();

            MotorPanel.ItemsSource = tracker.Motors;
            TofPanel.ItemsSource = tracker.Tofs;

            timer.Interval = TimeSpan.FromMilliseconds(TickMs);
            timer.Tick += Tick;
            timer.Start();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            RefreshPorts();
            DeviceInfoText.Text = "选择端口后点“打开设备”。设备为 CANable2（CDC 串口）。";
        }

        private void Window_Closed(object? sender, EventArgs e)
        {
            timer.Stop();
            device.Dispose();
        }

        private void RefreshPorts_Click(object sender, RoutedEventArgs e)
        {
            RefreshPorts();
        }

        private void RefreshPorts()
        {
            string? previous = PortBox.SelectedItem as string;
            PortBox.ItemsSource = SlcanDevice.GetPortNames();

            if (previous != null && PortBox.Items.Contains(previous))
            {
                PortBox.SelectedItem = previous;
            }
            else if (PortBox.Items.Count > 0)
            {
                PortBox.SelectedIndex = 0;
            }
            else
            {
                StatusText.Text = "没有找到串口，请确认设备已插好";
            }
        }

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (device.IsOpen)
            {
                device.Close();
                ConnectButton.Content = "打开设备";
                DeviceInfoText.Text = "已断开";
                return;
            }

            if (PortBox.SelectedItem is not string portName || portName.Length == 0)
            {
                DeviceInfoText.Text = "请先选择一个串口";
                return;
            }

            byte code = BitrateBox.SelectedIndex switch
            {
                0 => 8,     /* 1 Mbps */
                1 => 6,     /* 500 kbps */
                2 => 5,     /* 250 kbps */
                3 => 4,     /* 125 kbps */
                4 => 3,     /* 100 kbps */
                5 => 2,     /* 50 kbps */
                _ => 8,
            };

            if (device.Open(portName, code, out string message))
            {
                ConnectButton.Content = "关闭设备";
            }

            DeviceInfoText.Text = message;        }

        private void ClearButton_Click(object sender, RoutedEventArgs e)
        {
            rows.Clear();
            sequence = 0;
            framesAtLastRate = 0;
            frameRate = 0;
            renderedRowCount = -1;
            device.DropQueue();
            device.ResetCounters();
            tracker.Reset();
            FrameGrid.ItemsSource = null;
            StatusText.Text = "已清空";
        }

        private void ExportButton_Click(object sender, RoutedEventArgs e)
        {
            if (rows.Count == 0)
            {
                StatusText.Text = "没有可导出的报文";
                return;
            }

            Microsoft.Win32.SaveFileDialog dialog = new()
            {
                Filter = "CSV 文件|*.csv",
                FileName = "can-log-" + DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture) + ".csv",
            };

            if (dialog.ShowDialog() != true)
            {
                return;
            }

            StringBuilder builder = new();
            builder.AppendLine("seq,time,id,kind,data,decoded");
            foreach (FrameRow row in rows)
            {
                builder.Append(row.Seq).Append(',')
                       .Append(row.Time).Append(',')
                       .Append(row.Id).Append(',')
                       .Append(Quote(row.Kind)).Append(',')
                       .Append(Quote(row.Data)).Append(',')
                       .Append(Quote(row.Text))
                       .AppendLine();
            }

            File.WriteAllText(dialog.FileName, builder.ToString(), Encoding.UTF8);
            StatusText.Text = "已导出 " + rows.Count + " 行 → " + dialog.FileName;
        }

        private void Tick(object? sender, EventArgs e)
        {
            TickOnce();
        }

        private void TickOnce()
        {
            if (device.IsOpen)
            {
                device.TrimQueue(MaxPendingFrames);

                int drained = 0;
                while ((drained < MaxDrainPerTick) && device.TryDequeue(out RawFrame frame))
                {
                    tracker.Apply(frame);
                    AppendRow(frame);
                    drained++;
                }

                /* 通道哑掉时自动重开。该固件的通道状态跨会话保留，上一次会话
                   发过 C（关闭）后，本次若没打开成功就会一直静默。 */
                if ((AutoRecoverBox.IsChecked == true) && (device.IdleMs > RecoverIdleMs))
                {
                    if (device.Kick())
                    {
                        recoverCount++;
                    }
                }
            }

            long nowTotal = tracker.TotalFrames;
            frameRate = (nowTotal - framesAtLastRate) * (1000.0 / TickMs);
            framesAtLastRate = nowTotal;

            RefreshGrid();
            RefreshStatus();
        }

        private void AppendRow(RawFrame frame)
        {
            Decoded decoded = CanDecoder.Decode(frame.Id, frame.Data);
            sequence++;

            if (rows.Count >= MaxRows)
            {
                rows.RemoveAt(0);
            }

            rows.Add(new FrameRow(
                sequence,
                FormatTime(frame.TickMs),
                "0x" + frame.Id.ToString("X3", CultureInfo.InvariantCulture),
                decoded.Kind,
                CanDecoder.Hex(frame.Data, 0, frame.Data.Length),
                decoded.Text));
        }

        private static string FormatTime(long tickMs)
        {
            TimeSpan span = TimeSpan.FromMilliseconds(tickMs);
            return span.ToString(@"mm\:ss\.fff", CultureInfo.InvariantCulture);
        }

        private static string Quote(string value)
        {
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        }

        private void RefreshGrid()
        {
            /* 只在行集变化时重建，否则每帧都换一次 ItemsSource 会把选中与滚动位置抖掉 */
            if (renderedRowCount == rows.Count)
            {
                return;
            }

            renderedRowCount = rows.Count;
            FrameGrid.ItemsSource = null;
            FrameGrid.ItemsSource = rows;
        }

        private void RefreshStatus()
        {
            StringBuilder builder = new();
            builder.Append(device.IsOpen ? "已连接" : "未连接");
            builder.Append("　总帧 ").Append(tracker.TotalFrames);
            builder.Append("　帧/秒 ").Append(frameRate.ToString("F0", CultureInfo.InvariantCulture));
            builder.Append("　字节 ").Append(device.RawBytes);
            builder.Append("　行 ").Append(device.LineCount);
            builder.Append("　异常行 ").Append(device.MalformedLines);
            builder.Append("　错误帧 ").Append(tracker.ErrorFrames);
            builder.Append("　丢弃 ").Append(device.DroppedFrames);

            if (recoverCount > 0)
            {
                builder.Append("　重开 ").Append(recoverCount);
            }

            if (device.Fault.Length > 0)
            {
                builder.Append("　✖ ").Append(device.Fault);
            }
            else if (device.IsOpen && (device.RawBytes == 0))
            {
                builder.Append("　⚠ 串口无任何字节：总线没流量，或通道未打开");
            }
            else if (device.IsOpen && (tracker.TotalFrames == 0))
            {
                builder.Append("　⚠ 有字节但无帧：帧格式解析异常，看“异常行”");
            }

            StatusText.Text = builder.ToString();
        }
    }
}