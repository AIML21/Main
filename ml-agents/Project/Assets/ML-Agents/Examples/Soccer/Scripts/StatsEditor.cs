using System.IO;
using UnityEngine;
using TMPro;
using Unity.Profiling;

public class DetailedStatsDisplay : MonoBehaviour
{
    public TextMeshProUGUI statsText; // Reference to the TextMeshProUGUI component in the Scroll View Content
    public RectTransform contentRect; // Reference to the RectTransform of the Content GameObject
    public Canvas statsCanvas; // Reference to the Canvas component to toggle visibility

    private const float updateInterval = 0.5f; // Update interval in seconds
    private float elapsedTime = 0f;
    private const float Padding = 20f; // Extra padding at the bottom of the content
    private float fps; // Stores the calculated frame rate

    // ProfilerRecorder objects for CPU and GPU metrics
    private ProfilerRecorder mainThreadTimeRecorder;
    private ProfilerRecorder renderThreadTimeRecorder;

    void OnEnable()
    {
        // Initialize ProfilerRecorder for CPU and GPU time
        mainThreadTimeRecorder = ProfilerRecorder.StartNew(ProfilerCategory.Internal, "Main Thread");
        renderThreadTimeRecorder = ProfilerRecorder.StartNew(ProfilerCategory.Render, "Render Thread");
    }

    void OnDisable()
    {
        // Dispose ProfilerRecorder objects to free resources
        mainThreadTimeRecorder.Dispose();
        renderThreadTimeRecorder.Dispose();
    }

    void Update()
    {
        // Check for Enter key press to toggle Canvas visibility
        if (Input.GetKeyDown(KeyCode.Return)) // Enter key
        {
            ToggleCanvasVisibility();
        }

        elapsedTime += Time.deltaTime;

        // Calculate FPS (Frame Per Second)
        fps = Mathf.Clamp(1f / Time.deltaTime, 0, Application.targetFrameRate > 0 ? Application.targetFrameRate : 60);

        if (elapsedTime >= updateInterval)
        {
            elapsedTime = 0f;

            // Gather and display stats
            UpdateStats();
        }
    }

    void UpdateStats()
    {
        // Manually calculate average time for CPU and GPU usage
        float mainThreadTimeMs = CalculateAverage(mainThreadTimeRecorder) / 1_000_000f; // Convert from nanoseconds to milliseconds
        float renderThreadTimeMs = CalculateAverage(renderThreadTimeRecorder) / 1_000_000f; // Convert from nanoseconds to milliseconds

        // Dynamically calculate the target frame time
        float targetFrameTimeMs = 1000f / (Application.targetFrameRate > 0 ? Application.targetFrameRate : 60);

        // Estimate CPU and GPU usage based on frame time
        float estimatedCpuUsage = Mathf.Clamp((mainThreadTimeMs / targetFrameTimeMs) * 100f, 0, 100);
        float estimatedGpuUsage = Mathf.Clamp((renderThreadTimeMs / targetFrameTimeMs) * 100f, 0, 100);

        // Top Section: Performance Metrics
        string performanceMetrics = $"<b>Performance Metrics</b>\n" +
                                    $"- Frame Rate: {fps:F1} FPS\n" +
                                    $"- Main Thread Time: {mainThreadTimeMs:F2} ms\n" +
                                    $"- Render Thread Time: {renderThreadTimeMs:F2} ms\n" +
                                    $"- Estimated CPU Usage: {estimatedCpuUsage:F1}%\n" +
                                    $"- Estimated GPU Usage: {estimatedGpuUsage:F1}%\n\n";

        // Memory
        string memoryStats = $"<b>Memory</b>\n" +
                             $"- System Memory: {SystemInfo.systemMemorySize} MB\n" +
                             $"- Graphics Memory: {SystemInfo.graphicsMemorySize} MB\n" +
                             $"- Mono Heap Size: {UnityEngine.Profiling.Profiler.GetMonoHeapSizeLong() / (1024 * 1024):F2} MB\n" +
                             $"- Allocated Memory: {UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong() / (1024 * 1024):F2} MB\n\n";

        // Rendering
        string renderingStats = $"<b>Rendering</b>\n" +
                                $"- Active Rendering Time: {renderThreadTimeMs:F2} ms\n\n";

        // Audio
        string audioStats = $"<b>Audio</b>\n" +
                            $"- Active Audio Sources: {FindObjectsOfType<AudioSource>().Length}\n\n";

        // System Information
        string systemInfo = $"<b>System Information</b>\n" +
                            $"- Operating System: {SystemInfo.operatingSystem}\n" +
                            $"- Processor: {SystemInfo.processorType} ({SystemInfo.processorCount} cores)\n" +
                            $"- Graphics Device: {SystemInfo.graphicsDeviceName}\n" +
                            $"- Graphics API: {SystemInfo.graphicsDeviceType}\n" +
                            $"- Screen Resolution: {Screen.currentResolution.width}x{Screen.currentResolution.height}@{Screen.currentResolution.refreshRateRatio.value}Hz\n" +
                            $"- Device Model: {SystemInfo.deviceModel}\n" +
                            $"- Unity Version: {Application.unityVersion}\n\n";

        // Combine all stats
        string stats = performanceMetrics + memoryStats + renderingStats + audioStats + systemInfo;

        // Update the UI text
        statsText.text = stats;

        // Adjust the content size to ensure scrolling works correctly
        UpdateContentSize();
    }

    float CalculateAverage(ProfilerRecorder recorder)
    {
        long sum = 0;
        int count = recorder.Count;
        for (int i = 0; i < count; i++)
        {
            sum += recorder.GetSample(i).Value; // Access the numeric value of the sample
        }
        return count > 0 ? (float)sum / count : 0f;
    }


    void UpdateContentSize()
    {
        // Adjust the Content RectTransform to fit the height of the text
        Vector2 newSize = new Vector2(contentRect.sizeDelta.x, statsText.preferredHeight + Padding);
        contentRect.sizeDelta = newSize;
    }

    void ToggleCanvasVisibility()
    {
        // Toggle the Canvas component's enabled state
        if (statsCanvas != null)
        {
            statsCanvas.enabled = !statsCanvas.enabled;
        }
    }

    void OnApplicationQuit()
    {
        // Save the last displayed stats to a text file when the application closes
        string filePath = Path.Combine(Application.persistentDataPath, "DetailedProfilerLog.txt");
        File.WriteAllText(filePath, statsText.text);
    }
}
