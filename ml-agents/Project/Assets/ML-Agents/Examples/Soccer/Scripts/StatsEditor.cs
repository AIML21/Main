using System.IO;
using UnityEngine;
using TMPro;

public class StatsEditor : MonoBehaviour
{
    public TextMeshProUGUI statsText; // Reference to the TextMeshProUGUI component in the Scroll View Content
    private float updateInterval = 0.5f; // Update interval in seconds
    private float elapsedTime = 0f;
    private string logData = ""; // String to store log data

    void Start()
    {
        statsText.text = ""; // Initialize the text
    }

    void Update()
    {
        elapsedTime += Time.deltaTime;

        if (elapsedTime >= updateInterval)
        {
            elapsedTime = 0f;

            // Gather detailed profiler stats
            string stats = $"<b>CPU Usage</b>\n" +
                           $"- Rendering: Not available in build\n" +
                           $"- Scripts: N/A\n" +
                           $"- Garbage Collector: {UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong() / (1024 * 1024)} MB\n\n" +

                           $"<b>Memory</b>\n" +
                           $"- Total Used Memory: {UnityEngine.Profiling.Profiler.GetTotalReservedMemoryLong() / (1024 * 1024)} MB\n" +
                           $"- Allocated Memory: {UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong() / (1024 * 1024)} MB\n" +
                           $"- Graphics Memory: {SystemInfo.graphicsMemorySize} MB\n" +
                           $"- System Memory: {SystemInfo.systemMemorySize} MB\n\n" +

                           $"<b>Audio</b>\n" +
                           $"- Playing Audio Sources: {FindObjectsOfType<AudioSource>().Length}\n\n" +

                           $"<b>System Info</b>\n" +
                           $"- Operating System: {SystemInfo.operatingSystem}\n" +
                           $"- Processor: {SystemInfo.processorType} ({SystemInfo.processorCount} cores)\n" +
                           $"- Graphics Device: {SystemInfo.graphicsDeviceName}\n" +
                           $"- Screen Resolution: {Screen.currentResolution.width}x{Screen.currentResolution.height}@{Screen.currentResolution.refreshRateRatio.value}Hz\n" +
                           $"- Unity Version: {Application.unityVersion}\n";

            // Append stats to the log data
            logData += $"[{System.DateTime.Now}] {stats}\n";

            // Update the UI text in the Scroll View
            statsText.text = logData;
        }
    }

    void OnApplicationQuit()
    {
        // Save the log data to a text file when the application closes
        string filePath = Path.Combine(Application.dataPath, "DetailedProfilerLog.txt");
        File.WriteAllText(filePath, logData);
    }
}
