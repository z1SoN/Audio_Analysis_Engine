// 项目 JSON 保存下次应恢复的测量设置，WAV 保存可重新计算的采样。读取文件时统一处理旧版本设置与真实声道数，避免每个分析器各自猜测数据来源。
using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;
using ApxRemix.Core.Models;
using NAudio.Wave;
using NAudio.Dmo;

namespace ApxRemix.Core.Storage;

/// <summary>工程配置持久化、WAV 音频读写（保留真实声道数元数据）与自定义扫频点表解析服务。</summary>
public static class ProjectFileService
{
    // 枚举用成员名保存，工程可以直接读懂后端、窗函数或触发选项。加载时仍根据 SchemaVersion 处理同名选项的历史含义。
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        WriteIndented = true,
        Converters = { new JsonStringEnumConverter() }
    };

    public static void SaveProject(string filePath, ProjectStateFile state)
    {
        string? dir = Path.GetDirectoryName(filePath);
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
        state.SavedAtUtc = DateTime.UtcNow;
        // 先把完整 JSON 写到目标目录内的临时文件。完成写入后才替换原文件，失败时清理临时文件，避免将未写完的 JSON 直接放到目标路径。
        string target = Path.GetFullPath(filePath);
        string pending = Path.Combine(Path.GetDirectoryName(target)!, "." + Guid.NewGuid().ToString("N") + ".tmp");
        try
        {
            using (var stream = new FileStream(pending, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                JsonSerializer.Serialize(stream, state, JsonOptions);
            File.Move(pending, target, overwrite: true);
        }
        finally { if (File.Exists(pending)) File.Delete(pending); }
    }

    public static ProjectStateFile LoadProject(string filePath)
    {
        if (!File.Exists(filePath))
            throw new FileNotFoundException("找不到指定的工程文件。", filePath);
        string json = File.ReadAllText(filePath);
        using var document = JsonDocument.Parse(json);
        var state = JsonSerializer.Deserialize<ProjectStateFile>(json, JsonOptions)
            ?? throw new InvalidDataException("工程文件格式无效。");
        // Before 2.3 the BlackmanHarris label meant the four-term window.
        // 新版本会给缺失的字段赋默认值。这里先检查旧文件实际写过哪些版本信息，才能判断该沿用哪种历史行为，避免重新打开项目后测量设置悄悄改变。
        if (!document.RootElement.TryGetProperty(nameof(ProjectStateFile.SchemaVersion), out _) ||
            !Version.TryParse(state.SchemaVersion, out var version) || version < new Version(2, 3))
        {
            if (state.FftWindow == FftWindowType.BlackmanHarris) state.FftWindow = FftWindowType.BlackmanHarris4;
            if (state.TransferFunction?.WindowType == FftWindowType.BlackmanHarris)
                state.TransferFunction.WindowType = FftWindowType.BlackmanHarris4;
            if (state.Analyzer is { } analyzer) analyzer.ResidualWeighting = analyzer.Weighting;
            if (state.Sweep?.AnalyzerFilter is { } sweepAnalyzer) sweepAnalyzer.ResidualWeighting = sweepAnalyzer.Weighting;
            state.SchemaVersion = "2.3";
        }
        // Early projects stored the chirp range and drive level only under Sweep.
        var root = document.RootElement;
        bool hasContinuous = root.TryGetProperty(nameof(state.ContinuousSweep), out var chirp) && chirp.ValueKind == JsonValueKind.Object;
        state.ContinuousSweep ??= new();
        // 旧工程可能只有步进扫频设置。逐个检查原 JSON 是否明确给出连续扫频字段，只有缺项才沿用旧值。
        if (state.Sweep is { } sweep)
        {
            if (!hasContinuous || !chirp.TryGetProperty(nameof(sweep.StartFrequencyHz), out _))
                state.ContinuousSweep.StartFrequencyHz = sweep.StartFrequencyHz;
            if (!hasContinuous || !chirp.TryGetProperty(nameof(sweep.StopFrequencyHz), out _))
                state.ContinuousSweep.StopFrequencyHz = sweep.StopFrequencyHz;
            if (!hasContinuous || !chirp.TryGetProperty(nameof(ContinuousSweepConfig.LevelDbfs), out _))
                state.ContinuousSweep.LevelDbfs = sweep.SweepLevelDbfs;
        }
        if (!root.TryGetProperty(nameof(state.AcousticResponse), out var acoustic) || acoustic.ValueKind != JsonValueKind.Object ||
            !acoustic.TryGetProperty(nameof(AcousticResponseConfig.Chirp), out var stimulus) || stimulus.ValueKind != JsonValueKind.Object)
        {
            // 旧工程声学页缺专用 Chirp 设置时，用恢复后的连续扫频设置建立默认激励。
            state.AcousticResponse ??= new();
            state.AcousticResponse.Chirp = state.ContinuousSweep.Clone();
        }
        return state;
    }

    public static int? WavBitDepth(string path) => WavWordFormat(path).Depth;
    public static (int? Depth, bool ExactPcm) WavWordFormat(string path)
    {
        // 位深和采样编码是两件事。例如同为 32 位的整数 PCM 和浮点样本，能否精确比较数字码字取决于编码。
        if (!Path.GetExtension(path).Equals(".wav",StringComparison.OrdinalIgnoreCase)) return (null,false);
        using var stream = new WaveFileReader(path);
        return (InputPrecision(stream.WaveFormat), SampleLayout(stream.WaveFormat).Encoding == WaveFormatEncoding.Pcm);
    }

    public static int InputPrecision(WaveFormat format)
    {
        if (format.Encoding != WaveFormatEncoding.Extensible) return format.BitsPerSample;
        // NAudio exposes the container width. The first extensible format field holds
        // the actual input precision (for example, 20 valid bits in a 24-bit frame).
        using var header=new MemoryStream();
        using var writer=new BinaryWriter(header);
        format.Serialize(writer);
        // 扩展 WAV 头可能声明容器宽度和有效位数。读取实际有效位，并把异常值回退到容器宽度。
        var bytes=header.ToArray();
        // Serialize 会在格式记录前写长度，valid bits 因而位于序列化缓冲偏移 22。容器仍决定每样本字节数和帧步长。
        int valid=bytes.Length>=24 ? BitConverter.ToUInt16(bytes,22) : 0;
        return valid>0 && valid<=format.BitsPerSample ? valid : format.BitsPerSample;
    }

    private static WaveFormat SampleLayout(WaveFormat format)
    {
        if (format is WaveFormatExtensible ext) return ext.ToStandardWaveFormat();
        // WaveFileReader preserves the original fmt extension in WaveFormatExtraData.
        // 扩展格式需按子类型还原为整数 PCM 或浮点，后续才知道每个容器中的位应该如何解释。
        if (format.Encoding == WaveFormatEncoding.Extensible && format is WaveFormatExtraData extra && format.ExtraSize >= 22)
        {
            var subtype = new Guid(extra.ExtraData.AsSpan(6, 16));
            if (subtype == AudioMediaSubtypes.MEDIASUBTYPE_PCM)
                return new WaveFormat(format.SampleRate, format.BitsPerSample, format.Channels);
            if (subtype == AudioMediaSubtypes.MEDIASUBTYPE_IEEE_FLOAT && format.BitsPerSample == 32)
                return WaveFormat.CreateIeeeFloatWaveFormat(format.SampleRate, format.Channels);
        }
        return format;
    }

    private static WaveStream OpenAudioFile(string path)
    {
        // 对可直接读取的 WAV 保留原始格式，其他情况交给解码器。这样整数 PCM 可以走后面保留低位的路径。
        if (Path.GetExtension(path).Equals(".wav", StringComparison.OrdinalIgnoreCase))
        {
            var wave = new WaveFileReader(path);
            // 扩展头先提取样本子类型。有原始整数或浮点语义时直接用 WAV 容器，否则通过音频解码器获得样本。
            var format = SampleLayout(wave.WaveFormat);
            if (format.Encoding is WaveFormatEncoding.Pcm or WaveFormatEncoding.IeeeFloat)
                return wave;
            wave.Dispose();
        }
        return new AudioFileReader(path);
    }

    public static (int SampleRate, int Channels, long Frames) AudioFileInfo(string path)
    {
        using var audio = OpenAudioFile(path);
        var format = audio.WaveFormat;
        // 一帧包括同一时刻所有通道的数据，所以帧数等于字节数除以每帧字节数 BlockAlign。
        return (format.SampleRate, format.Channels, audio.Length / format.BlockAlign);
    }

    /// <summary>读取 WAV 文件并返回双通道数据、采样率及原始物理声道数（区分单声道复制与真双声道）。</summary>
    public static (double[] Left, double[] Right, int SampleRate, int SourceChannels) ReadStereoWavWithMeta(string filePath,int first=0,int second=1)
    {
        if (!File.Exists(filePath))
            throw new FileNotFoundException("找不到指定的 WAV 音频文件。", filePath);

        using var stream = OpenAudioFile(filePath);
        // Extensible PCM uses the same sample layout. Read the interleaved frames
        // directly so multi-channel files keep their channel order without Windows ACM.
        using var pcm = stream.WaveFormat.Encoding == WaveFormatEncoding.Extensible
            ? new RawSourceWaveStream(stream, SampleLayout(stream.WaveFormat)) : null;
        var reader = stream as ISampleProvider ?? ((WaveStream?)pcm ?? stream).ToSampleProvider();
        int sampleRate = reader.WaveFormat.SampleRate;
        int channels = Math.Max(1, reader.WaveFormat.Channels);

        // 单声道只提供一份独立观测，可以复制到两个逻辑槽，但在返回元数据中仍要注明实际只用了一路。
        if (channels==1 && second==1) second=0;
        if (first<0 || first>=channels || second<0 || second>=channels)
            throw new InvalidDataException($"The WAV has {channels} channels. Check the input channel map.");
        // 32 位整数经 float 中转可能丢失最低几位。按原始字节读入，再以 double 除以 2³¹，给数字精度测量保留这些低位。
        if (SampleLayout(stream.WaveFormat) is { Encoding: WaveFormatEncoding.Pcm, BitsPerSample:32 })
        {
            // ISampleProvider converts PCM32 to float, which discards up to eight low bits.
            var samples = new List<double>();
            var others = new List<double>();
            byte[] bytes = new byte[4096 * stream.WaveFormat.BlockAlign];
            int n;
            // 每块只遍历完整交错帧。32 位整数码精确读入 double，除以 2³¹ 后保留二进制低位。例如 0xC0000000 对应 −0.5。
            while ((n=stream.Read(bytes,0,bytes.Length))>0)
                for (int offset=0; offset+stream.WaveFormat.BlockAlign<=n; offset+=stream.WaveFormat.BlockAlign)
                {
                    samples.Add(System.Buffers.Binary.BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(offset+4*first))/2147483648.0);
                    others.Add(System.Buffers.Binary.BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(offset+4*second))/2147483648.0);
                }
            return (samples.ToArray(),others.ToArray(),sampleRate,first==second ? 1 : channels);
        }
        var leftList = new List<double>();
        var rightList = new List<double>();
        var chunk = new float[4096 * channels];

        int read;
        while ((read = reader.Read(chunk, 0, chunk.Length)) > 0)
        {
            int frames = read / channels;
            // 一般路径读的是通道交错数据。每推进一帧就取映射指定的两个位置，放入两个按时间排列的数组。
            for (int i = 0; i < frames; i++)
            {
                int idx = i * channels;
                double l = chunk[idx+first];
                // 左、右逻辑槽同时选中一个源声道时，两数组在本帧写入相同值。独立性由返回的通道元数据表达。
                double r = chunk[idx+second];
                leftList.Add(l);
                rightList.Add(r);
            }
        }

        // 映射后如果两个数组取的是同一源位置，元数据报告一路独立观测。原文件虽可含多个声道，重复映射也不能做双通道测量。
        return (leftList.ToArray(), rightList.ToArray(), sampleRate, first==second ? 1 : channels);
    }

    public static (double[] Left,double[] Right,int SampleRate,int SourceChannels) ReadMeasurementInput(string path,SignalPathConfig? config,int? playbackRate=null)
    {
        var data=ReadStereoWavWithMeta(path,config?.AudioInputCh1 ?? 0,config?.AudioInputCh2 ?? 1);
        // 这里覆盖的是样本时间标尺，本段没有生成插值样本；输入 EQ 随后按实际采用的采样率处理两路。
        if (playbackRate.HasValue) data.SampleRate=playbackRate.Value;
        new ApxRemix.Core.Dsp.SignalPathEqualizer().Process(config?.InputEq,data.SampleRate,data.Left,data.Right);
        return data;
    }

    /// <summary>读取 WAV 文件为双精度浮点双通道数组（向后兼容签名）。</summary>
    public static (double[] Left, double[] Right, int SampleRate) ReadStereoWav(string filePath)
    {
        var (l, r, fs, _) = ReadStereoWavWithMeta(filePath);
        return (l, r, fs);
    }

    /// <summary>写入标准 32-bit IEEE Float 立体声 WAV 文件。</summary>
    public static void WriteStereoFloatWav(string filePath, ReadOnlySpan<double> left, ReadOnlySpan<double> right, int sampleRate, int iterations = 1)
    {
        int count = Math.Min(left.Length, right.Length);
        // 写入前按容器容量核对总长度。写双声道时先按 L、R 交错成块，再写入文件，重复次数不要求在内存复制整段。
        if (iterations < 1 || (long)count * iterations > (uint.MaxValue - 128) / (2 * sizeof(float)))
            throw new ArgumentException("The requested waveform exceeds the WAV file size limit.");
        string? dir = Path.GetDirectoryName(filePath);
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);

        // 写回时把时间轴放进 WAV 头。每帧按左、右各一个 float32 排列，占八字节，循环次数计入总长度。
        var fmt = WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, 2);
        using var writer = new WaveFileWriter(filePath, fmt);

        var interleaved = new float[Math.Min(count, 4096) * 2];
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int offset = 0; offset < count; offset += 4096)
            {
                int block = Math.Min(4096, count - offset);
                for (int i = 0; i < block; i++)
                {
                    interleaved[i * 2] = (float)left[offset + i];
                    interleaved[i * 2 + 1] = (float)right[offset + i];
                }
                writer.WriteSamples(interleaved, 0, block * 2);
            }
        }
    }

    /// <summary>写入标准 32-bit IEEE Float 单声道 WAV 文件（用于单声道测试与开环 Chirp 导出）。</summary>
    public static void WriteMonoFloatWav(string filePath, ReadOnlySpan<double> mono, int sampleRate, int iterations=1)
    {
        if (iterations<1 || (long)mono.Length*iterations*sizeof(float)>uint.MaxValue-128)
            throw new ArgumentException("The requested waveform exceeds the WAV file size limit.");
        string? dir = Path.GetDirectoryName(filePath);
        // 单声道文件每帧只有一个样本。固定大小的转换缓冲逐块写盘，额外内存不会随迭代次数增长。
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);

        var fmt = WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, 1);
        using var writer = new WaveFileWriter(filePath, fmt);

        var buf = new float[Math.Min(mono.Length, 4096)];
        for (int iteration=0; iteration<iterations; iteration++)
        {
            for (int offset=0; offset<mono.Length; offset+=buf.Length)
            {
                int block = Math.Min(buf.Length, mono.Length-offset);
                for (int i=0; i<block; i++) buf[i]=(float)mono[offset+i];
                writer.WriteSamples(buf,0,block);
            }
        }
    }

    public static double[] ParseCustomSweepPoints(string textOrFilePath)
    {
        string raw = File.Exists(textOrFilePath) ? File.ReadAllText(textOrFilePath) : textOrFilePath;
        var list = new List<double>();
        // 表可以来自文件也可以直接输入。按不依赖系统地区的数字格式解析，逗号、空白和换行都可分隔测点。
        foreach (string token in raw.Split(['\r', '\n', ',', ';', '\t', ' '], StringSplitOptions.RemoveEmptyEntries))
        {
            // 非数字项被跳过，成功解析的项保留原顺序。范围、单位、是否需要排序由调用测量的流程处理。
            if (double.TryParse(token.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out double val))
                list.Add(val);
        }
        return list.ToArray();
    }
}
