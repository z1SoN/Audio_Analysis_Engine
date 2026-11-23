using ApxRemix.Core.Models;

namespace ApxRemix.Core.Storage;

public enum FileInputRateMode { MatchAllFiles, FirstFile, FixedRate }

// 计划只冻结文件顺序和解释用采样率。统一时间轴时，同一组样本可能有不同的播放时间长度。
public sealed record InputFilePlan(string[] Paths,int Rate)
{
    public static InputFilePlan Open(IEnumerable<string> files,string? single,FileInputRateMode mode,int fixedRate)
    {
        string[] paths=files.ToArray();
        // 优先按输入列表的顺序分析。列表为空才退回旧工程里的单文件设置，避免把兼容字段额外加进播放列表。
        if (paths.Length==0 && !string.IsNullOrWhiteSpace(single)) paths=[single];
        if (paths.Length is 0 or >16) throw new InvalidDataException("Select between one and sixteen input files.");
        int firstRate=0;
        foreach(string path in paths)
        {
            // 打开文件并查看格式信息，先检查是否有完整帧，再记住首文件采样率。在要求文件采样率一致的模式下，采样率冲突在开始分析前报告。
            if (!File.Exists(path)) throw new FileNotFoundException($"Input file not found: {path}",path);
            var info=ProjectFileService.AudioFileInfo(path);
            if(info.Frames==0) throw new InvalidDataException($"Input file contains no samples: {path}");
            int rate=info.SampleRate;
            if(firstRate==0) firstRate=rate;
            // MatchAllFiles 要求每份文件头的采样率一致。FirstFile 用首份速率，FixedRate 用用户指定的速率，两种都允许重新解释其他文件的样本间隔。
            if(mode==FileInputRateMode.MatchAllFiles && rate!=firstRate)
                throw new InvalidDataException("The input files have different sample rates. Choose First file or Fixed rate to interpret them at one rate.");
        }
        // 固定采样率模式使用显式 Fs，其余以首个文件的 Fs 建立时间轴。之后样本序号除以这个速率才得到秒。
        if(mode==FileInputRateMode.FixedRate && fixedRate is <8000 or >768000)
            throw new InvalidDataException("Fixed playback rate must be between 8000 and 768000 Hz.");
        return new(paths,mode==FileInputRateMode.FixedRate ? fixedRate : firstRate);
    }
}
