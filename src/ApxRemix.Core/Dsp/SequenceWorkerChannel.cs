using System.Diagnostics;

namespace ApxRemix.Core.Dsp;

internal static class SequenceWorkerChannel
{
    // The caller serializes all requests for this process. A failed exchange discards
    // both pipe ends, so a partial reply cannot be mistaken for the following frame.
    // 请求和应答均以字节计。调用方用锁串行访问同一进程，才能让一条回复对应一条请求。
    internal static byte[] Run(ref Process? worker, string executable, string? libraryDirectory,
        MemoryStream message, int responseBytes)
    {
        var reply = new byte[responseBytes];
        try
        {
            // 首次使用或上个进程已退出时重新启动，重定向标准流作为二进制数据通道。
            if (worker == null || worker.HasExited)
            {
                worker?.Dispose();
                worker = null;
                var start = new ProcessStartInfo(executable) {
                    WorkingDirectory = libraryDirectory ?? AppContext.BaseDirectory,
                    UseShellExecute = false, CreateNoWindow = true,
                    RedirectStandardInput = true, RedirectStandardOutput = true
                };
                // 指定库目录时把它置于子进程 PATH 首位，便于按该工作表位数加载运行库。
                if (libraryDirectory != null)
                    start.Environment["PATH"] = libraryDirectory + Path.PathSeparator + start.Environment["PATH"];
                worker = Process.Start(start) ?? throw new IOException("无法启动 Sequence 数值进程。");
            }
            // 只发送消息中已写入的字节，要求 30 秒内取得预期长度的应答。
            Exchange(worker, message.GetBuffer().AsMemory(0, checked((int)message.Length)), reply)
                .WaitAsync(TimeSpan.FromSeconds(30)).GetAwaiter().GetResult();
        }
        catch
        {
            // 异常会令进程和本条协议的边界不再可靠，清除引用并终止残留进程，让下一次请求重新启动。
            var failed = worker;
            worker = null;
            try { if (failed is { HasExited: false }) failed.Kill(); }
            catch (InvalidOperationException) { /* The worker exited before Kill. */ }
            finally { failed?.Dispose(); }
            throw;
        }
        return reply;
    }

    // 写完整条请求后刷新，使另一端能立即开始；ReadExactlyAsync 持续读取直至应答齐全或遇到错误。
    private static async Task Exchange(Process worker, ReadOnlyMemory<byte> request, Memory<byte> reply)
    {
        var input = worker.StandardInput.BaseStream;
        await input.WriteAsync(request).ConfigureAwait(false);
        await input.FlushAsync().ConfigureAwait(false);
        await worker.StandardOutput.BaseStream.ReadExactlyAsync(reply).ConfigureAwait(false);
    }
}
