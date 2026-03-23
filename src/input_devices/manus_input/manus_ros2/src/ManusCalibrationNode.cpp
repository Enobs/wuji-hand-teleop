/**
 * MANUS Glove Calibration Node
 *
 * Standalone ROS2 node for calibrating MANUS gloves via SDK API.
 * Runs the calibration procedure interactively in the terminal,
 * then saves the .mcal file for future use.
 *
 * Usage:
 *   ros2 run manus_ros2 manus_calibration
 */

#include "ManusSDK.h"
#include "ManusSDKTypes.h"
#include "ManusSDKTypeInitializers.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <string>
#include <atomic>

using namespace std::chrono_literals;

// ─────────────────────── Landscape callback ───────────────────────

static std::mutex g_LandscapeMtx;
static Landscape g_Landscape;
static std::atomic<bool> g_LandscapeReady{false};

static void OnLandscapeCallback(const Landscape* const p_Landscape)
{
    std::lock_guard<std::mutex> lock(g_LandscapeMtx);
    g_Landscape = *p_Landscape;
    g_LandscapeReady = true;
}

// ─────────────────────── Helper: list gloves ───────────────────────

struct GloveInfo {
    uint32_t id;
    std::string side;
};

static std::vector<GloveInfo> GetConnectedGloves()
{
    std::vector<GloveInfo> gloves;
    std::lock_guard<std::mutex> lock(g_LandscapeMtx);
    for (uint32_t i = 0; i < g_Landscape.gloveDevices.gloveCount; i++) {
        GloveInfo g;
        g.id = g_Landscape.gloveDevices.gloves[i].id;
        g.side = (g_Landscape.gloveDevices.gloves[i].side == Side_Left) ? "Left" : "Right";
        gloves.push_back(g);
    }
    return gloves;
}

// ─────────────────────── Save calibration to file ───────────────────────

static bool SaveCalibrationToFile(uint32_t gloveId, const std::string& side)
{
    // Get calibration data size
    uint32_t calSize = 0;
    auto ret = CoreSdk_GetGloveCalibrationSize(gloveId, &calSize);
    if (ret != SDKReturnCode_Success || calSize == 0) {
        std::cerr << "[ERROR] Failed to get calibration size for glove 0x"
                  << std::hex << gloveId << std::dec << std::endl;
        return false;
    }

    // Get calibration bytes
    std::vector<unsigned char> calData(calSize);
    ret = CoreSdk_GetGloveCalibration(calData.data(), calSize);
    if (ret != SDKReturnCode_Success) {
        std::cerr << "[ERROR] Failed to get calibration data" << std::endl;
        return false;
    }

    // Save to file
    std::string packageDir;
    try {
        packageDir = ament_index_cpp::get_package_share_directory("manus_ros2");
    } catch (...) {
        packageDir = ".";
    }

    std::string filename = packageDir + "/calibration/" + side + "QuantumMetaglove.mcal";
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "[ERROR] Cannot write to: " << filename << std::endl;
        // Try current directory as fallback
        filename = side + "QuantumMetaglove.mcal";
        file.open(filename, std::ios::binary);
        if (!file) {
            std::cerr << "[ERROR] Cannot write to fallback: " << filename << std::endl;
            return false;
        }
    }

    file.write(reinterpret_cast<char*>(calData.data()), calSize);
    file.close();
    std::cout << "[OK] Calibration saved to: " << filename << " (" << calSize << " bytes)" << std::endl;
    return true;
}

// ─────────────────────── Calibrate one glove ───────────────────────

static bool CalibrateGlove(uint32_t gloveId, const std::string& side)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Calibrating " << side << " glove (ID: 0x"
              << std::hex << gloveId << std::dec << ")" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Get number of calibration steps
    GloveCalibrationArgs calArgs;
    calArgs.gloveId = gloveId;

    uint32_t numSteps = 0;
    auto ret = CoreSdk_GloveCalibrationGetNumberOfSteps(calArgs, &numSteps);
    if (ret != SDKReturnCode_Success || numSteps == 0) {
        std::cerr << "[ERROR] Failed to get calibration steps (ret=" << ret << ")" << std::endl;
        return false;
    }
    std::cout << "Calibration requires " << numSteps << " steps.\n" << std::endl;

    // Start calibration
    bool result = false;
    ret = CoreSdk_GloveCalibrationStart(calArgs, &result);
    if (ret != SDKReturnCode_Success || !result) {
        std::cerr << "[ERROR] Failed to start calibration (ret=" << ret << ")" << std::endl;
        return false;
    }

    // Execute each step
    for (uint32_t step = 0; step < numSteps; step++) {
        GloveCalibrationStepArgs stepArgs;
        stepArgs.gloveId = gloveId;
        stepArgs.stepIndex = step;

        // Get step info
        GloveCalibrationStepData stepData;
        ret = CoreSdk_GloveCalibrationGetStepData(stepArgs, &stepData);
        if (ret != SDKReturnCode_Success) {
            std::cerr << "[ERROR] Failed to get step " << step << " data" << std::endl;
            CoreSdk_GloveCalibrationStop(calArgs, &result);
            return false;
        }

        std::cout << "--- Step " << (step + 1) << "/" << numSteps << " ---" << std::endl;
        std::cout << "  Title:       " << stepData.title << std::endl;
        std::cout << "  Description: " << stepData.description << std::endl;
        if (stepData.time > 0) {
            std::cout << "  Duration:    " << stepData.time << " seconds" << std::endl;
        } else {
            std::cout << "  Duration:    Continuous (repeat until complete)" << std::endl;
        }

        std::cout << "\nPress ENTER when ready to start this step..." << std::endl;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // Execute step
        ret = CoreSdk_GloveCalibrationStartStep(stepArgs, &result);
        if (ret != SDKReturnCode_Success || !result) {
            std::cerr << "[ERROR] Step " << (step + 1) << " failed (ret=" << ret << ")" << std::endl;
            CoreSdk_GloveCalibrationStop(calArgs, &result);
            return false;
        }

        std::cout << "  Step " << (step + 1) << " completed!\n" << std::endl;
    }

    // Finish calibration
    ret = CoreSdk_GloveCalibrationFinish(calArgs, &result);
    if (ret != SDKReturnCode_Success || !result) {
        std::cerr << "[ERROR] Failed to finish calibration" << std::endl;
        return false;
    }

    std::cout << "[OK] Calibration completed for " << side << " glove!" << std::endl;

    // Save to file
    SaveCalibrationToFile(gloveId, side);

    return true;
}

// ─────────────────────── Main ───────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("manus_calibration");

    std::cout << R"(
  __  __    _    _   _ _   _ ____
 |  \/  |  / \  | \ | | | | / ___|
 | |\/| | / _ \ |  \| | | | \___ \
 | |  | |/ ___ \| |\  | |_| |___) |
 |_|  |_/_/   \_\_| \_|\___/|____/
      Glove Calibration Tool
    )" << std::endl;

    // Initialize SDK (Integrated mode)
    auto initRet = CoreSdk_InitializeIntegrated();
    if (initRet != SDKReturnCode_Success) {
        std::cerr << "[ERROR] Failed to initialize SDK (ret=" << initRet << ")" << std::endl;
        return 1;
    }

    // Register landscape callback
    CoreSdk_RegisterCallbackForLandscapeStream(OnLandscapeCallback);

    // Set coordinate system
    CoordinateSystemVUH coordSys;
    CoordinateSystemVUH_Init(&coordSys);
    bool worldSpace = false;
    CoreSdk_InitializeCoordinateSystemWithVUH(coordSys, worldSpace);

    std::cout << "[INFO] SDK initialized. Connecting to host..." << std::endl;

    // Connect (same as manus_data_publisher: LookForHosts → ConnectToHost)
    bool connected = false;
    for (int attempt = 0; attempt < 30; attempt++) {
        auto lookRet = CoreSdk_LookForHosts(5, false);
        if (lookRet != SDKReturnCode_Success) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        uint32_t numHosts = 0;
        CoreSdk_GetNumberOfAvailableHostsFound(&numHosts);
        if (numHosts == 0) {
            std::cout << "[INFO] No hosts found, retrying... (" << attempt + 1 << ")" << std::endl;
            std::this_thread::sleep_for(1s);
            continue;
        }

        auto hosts = std::make_unique<ManusHost[]>(numHosts);
        CoreSdk_GetAvailableHostsFound(hosts.get(), numHosts);
        auto connectRet = CoreSdk_ConnectToHost(hosts[0]);
        if (connectRet == SDKReturnCode_Success) {
            std::cout << "[INFO] Connected to host!" << std::endl;
            connected = true;
            break;
        }
        std::this_thread::sleep_for(1s);
    }

    if (!connected) {
        std::cerr << "[ERROR] Failed to connect after 30 attempts." << std::endl;
        CoreSdk_ShutDown();
        return 1;
    }

    std::cout << "[INFO] Waiting for glove detection..." << std::endl;

    // Wait for landscape data with gloves
    int waitCount = 0;
    while (!g_LandscapeReady || GetConnectedGloves().empty()) {
        std::this_thread::sleep_for(500ms);
        waitCount++;
        if (waitCount % 10 == 0) {
            std::cout << "[INFO] Still waiting for gloves... (" << waitCount / 2 << "s)" << std::endl;
        }
        if (waitCount > 60) {  // 30 seconds timeout
            std::cerr << "[ERROR] No gloves detected after 30 seconds. Check connection." << std::endl;
            CoreSdk_ShutDown();
            return 1;
        }
    }

    // List connected gloves
    auto gloves = GetConnectedGloves();
    std::cout << "\n[INFO] Detected " << gloves.size() << " glove(s):" << std::endl;
    for (size_t i = 0; i < gloves.size(); i++) {
        std::cout << "  [" << i << "] " << gloves[i].side << " (ID: 0x"
                  << std::hex << gloves[i].id << std::dec << ")" << std::endl;
    }

    // Calibrate each glove
    for (auto& glove : gloves) {
        std::cout << "\nCalibrate " << glove.side << " glove? [Y/n]: ";
        std::string input;
        std::getline(std::cin, input);
        if (input.empty() || input[0] == 'Y' || input[0] == 'y') {
            CalibrateGlove(glove.id, glove.side);
        } else {
            std::cout << "Skipping " << glove.side << " glove." << std::endl;
        }
    }

    std::cout << "\n[DONE] Calibration tool finished." << std::endl;

    CoreSdk_ShutDown();
    rclcpp::shutdown();
    return 0;
}
