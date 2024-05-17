/**
* Code adapted from the main.cpp file of the RunCalibrationApi executable from the PAI Calibration tool.
* This code modifies the main function to become a callable named RunCalibrationApi that accepts
* a function callback, which can be used to send progress information to the Control Panel.
* 
* Repo: https://microsoft.visualstudio.com/Analog/_git/vision.presence.babylon?path=/Holoportation/Tools/Calibration/RunCalibrationApi
* Commit: f7e69cc2
**/

#include "PAI/InputParameters.h"
#include "PAI/Metrics.h"
#include "Logger.h"

#include <3DReconstruction/3DReconstruction.h>
#include <Common/CalibrationDataLoader.h>
#include <Common/CalibrationJsonSerialization.h>
#include <Common/Visualization.h>

#include <Useful/Stopwatch.h>

#include <iostream>

using namespace Holoportation::API;

void SaveCalibrationParameters(const std::vector<RgbdDevice>& devices, const std::filesystem::path& outDir)
{
    // Convert calibration to the format that reconstruction is expecting.
    const size_t numberOfDevices = devices.size();
    std::vector<Holoportation::API::Reconstruction::InputRGBDCamera> inputCameras(numberOfDevices);
    for (size_t deviceIdx = 0; deviceIdx < numberOfDevices; ++deviceIdx)
    {
        inputCameras[deviceIdx].depth = devices[deviceIdx].DepthCamera;
        inputCameras[deviceIdx].rgb = devices[deviceIdx].RgbCamera;
        const auto worldToDevice = Holoportation::FromAPI(*devices[deviceIdx].WorldToDevice);
        const auto deviceToDepth = Holoportation::FromAPI(devices[deviceIdx].DepthCamera.parentToView);
        const auto deviceToRgb = Holoportation::FromAPI(devices[deviceIdx].RgbCamera.parentToView);
        inputCameras[deviceIdx].depth.parentToView = Holoportation::ToAPI(deviceToDepth * worldToDevice);
        inputCameras[deviceIdx].rgb.parentToView = Holoportation::ToAPI(deviceToRgb * worldToDevice);
        inputCameras[deviceIdx].serialNumber = devices[deviceIdx].DeviceSerialNumber;
    }

    // Save in all useful formats.
    Write(outDir / "devicesCalibration.json", "devicesCalibration", devices);
    Write(outDir / "inputCameras.json", "inputCameras", inputCameras);
    LegacyMW::SaveCameras(inputCameras, outDir);
}

void SaveCalibrationParameters(const std::vector<StereoIrPod>& devices, const std::filesystem::path& outDir)
{
    // Convert calibration to the format that reconstruction is expecting.
    const size_t numberOfDevices = devices.size();
    std::vector<Holoportation::API::Reconstruction::InputStereoIRPod> inputCameras(numberOfDevices);
    for (size_t deviceIdx = 0; deviceIdx < numberOfDevices; ++deviceIdx)
    {
        inputCameras[deviceIdx].irReference = devices[deviceIdx].IrReference;
        inputCameras[deviceIdx].irSecondView = devices[deviceIdx].IrSecondView;
        inputCameras[deviceIdx].rgb = devices[deviceIdx].RgbCamera;
        const auto worldToDevice = Holoportation::FromAPI(*devices[deviceIdx].WorldToDevice);
        const auto deviceToIrReference = Holoportation::FromAPI(devices[deviceIdx].IrReference.parentToView);
        const auto deviceToIrSecondView = Holoportation::FromAPI(devices[deviceIdx].IrSecondView.parentToView);
        const auto deviceToRgb = Holoportation::FromAPI(devices[deviceIdx].RgbCamera.parentToView);
        inputCameras[deviceIdx].irReference.parentToView = Holoportation::ToAPI(deviceToIrReference * worldToDevice);
        inputCameras[deviceIdx].irSecondView.parentToView = Holoportation::ToAPI(deviceToIrSecondView * worldToDevice);
        inputCameras[deviceIdx].rgb.parentToView = Holoportation::ToAPI(deviceToRgb * worldToDevice);
    }

    // Save in all useful formats.
    Write(outDir / "devicesCalibration.json", "devicesCalibration", devices);
    Write(outDir / "inputCameras.json", "inputCameras", inputCameras);
}

template <typename DeviceType, typename SensorInputType, typename CalibrationDataLoaderType>
std::vector<DeviceType> Calibrate(
    const InputParameters& params,
    CalibrationDataLoaderType& dataLoader,
    Metrics::ClipMetrics& clipMetrics,
    Metrics& metrics,
    void* progressState,
    std::function<void(void*, uint32_t, uint32_t)> ProgressCallback, 
    std::function<void(void*, float, float, float)> CalibrationCompletedCallback)
{
    if (dataLoader.GetRandomSeed() != CalibrationDataInputParameters::DontRandomize)
        *LOGGER() << Logger::Info << "Random seed: " << dataLoader.GetRandomSeed() << Logger::endl;

    dataLoader.RewindAndMaybeShuffleData();
    std::vector<DeviceType> devices = dataLoader.GetDeviceCalibrations();

    Holoportation::Visualization::CalibrationVisualizer debugVisualizer(params.visualizationParams);

    Useful::Stopwatch stopwatch;
    // Initialize calibration and FrameState objects.
    Calibration calibration(params.calibrationParams, devices);
    Calibration::FrameState frameState(calibration);

    calibration.SetConfig(params.configJsonPath);

    uint32_t numberOfProcessedFrames = 0;
    *LOGGER() << Logger::Info << "Calibration frames start from frame " << params.startFrame << " and use every "
        << params.keepFrameRate << "'th frame." << Logger::endl;
    std::function<bool(uint32_t)> ShouldProcess;
    if (params.c_endFrameNotSet == params.endFrame)
        ShouldProcess = [&](uint32_t) { return !calibration.IsDataCaptureComplete(); };
    else
        ShouldProcess = [&](uint32_t frameIdx) { return frameIdx < params.endFrame; };
    // Capture Loop:
    for (uint32_t frameIndex = params.startFrame; ShouldProcess(frameIndex); frameIndex += params.keepFrameRate)
    {
        const std::vector<SensorInputType>& sensorsData = dataLoader.GetSensorsData(frameIndex);
        if (sensorsData.empty())
            break;

        const auto frameQuality = calibration.ProcessSensorData<SensorInputType>(sensorsData, frameState);
        if (frameQuality == Calibration::FrameQualityFlags::SubframesAreOutOfSync)
            *LOGGER() << Logger::Info << "Frames got rejected due to not being well synchronized." << Logger::endl;

        debugVisualizer.ProcessFrameState(sensorsData, frameState);
        *LOGGER() << Logger::Info << "Processed frame " << frameIndex << "\r" << Logger::endl;
        
        ProgressCallback(progressState, frameIndex, params.endFrame);

        debugVisualizer.MaybeSaveFrameDebugArtifacts(frameIndex);
        ++numberOfProcessedFrames;
    }
    *LOGGER() << Logger::Info << Logger::endl;
    clipMetrics.ExecutionTime.Total = static_cast<float>(stopwatch.ElapsedSeconds());
    clipMetrics.ExecutionTime.PreprocessingPerFrame = clipMetrics.ExecutionTime.Total / numberOfProcessedFrames;
    *LOGGER() << Logger::Info << "Data processing time per frame: "
        << std::round(clipMetrics.ExecutionTime.PreprocessingPerFrame * 1000.f) << " milliseconds." << Logger::endl;
    debugVisualizer.MaybeSaveSessionDebugArtifacts();

    // Execute calibration on the collected data.
    stopwatch.Restart();
    try
    {
        const auto csTransforms = calibration.CalibrateDevices<DeviceType>(devices, clipMetrics.Accuracy);
        *LOGGER() << Logger::Info << "Performed Coordinate System Transforms:" << Logger::endl
            << "\ty-axis is " << (csTransforms.AlignYAxisToVerticalDown ? "" : "NOT ")
            << "aligned with the vertical down direction." << Logger::endl
            << "\tworld is " << (csTransforms.RotateAboutYAxisToAlignWithAudience ? "" : "NOT ")
            << "rotated about y-axis so that audience view direction is aligned with z-axis." << Logger::endl
            << "\tworld is " << (csTransforms.TranslateCenterToCamerasOpticalAxesIntersection ? "" : "NOT ")
            << "translated so that (0, 0, 0) is in the intersection of the camera optical axes." << Logger::endl
            << "\tworld is " << (csTransforms.OffsetCenterToFloorPlane ? "" : "NOT ")
            << "offset so that (0, 0, 0) is in the floor plane." << Logger::endl;
    }
    catch (CalibrationCaptureIncompleteException& exception)
    {
        *LOGGER() << Logger::Error << exception.what() << Logger::endl;
    }

    clipMetrics.ExecutionTime.Optimization = static_cast<float>(stopwatch.ElapsedSeconds());
    *LOGGER() << Logger::Info << "CalibrateDevices took: " << std::round(clipMetrics.ExecutionTime.Optimization) << " seconds."
        << Logger::endl;
    clipMetrics.ExecutionTime.Total += clipMetrics.ExecutionTime.Optimization;

    const auto annotatedFloor = dataLoader.GetAnnotatedFloor(devices);
    if (annotatedFloor)
    {
        clipMetrics.ClipFloorMetric = Metrics::ComputeClipFloorMetric(
            annotatedFloor->AnnotatedWorldToDepth, devices[annotatedFloor->MiddleDeviceIndex]);
    }

    debugVisualizer.MaybeSavePostCalibrationArtifacts<DeviceType>(clipMetrics.Accuracy);
    debugVisualizer.MaybeSaveCoordinateSystems<DeviceType>(devices, annotatedFloor);

    clipMetrics.RunType = Metrics::RunType::Train;
    clipMetrics.ClipId = params.calibrationDataInput.MkvsDir.filename();
    clipMetrics.DeviceCount = static_cast<uint32_t>(devices.size());
    clipMetrics.NumberOfProcessedFrames = numberOfProcessedFrames;
    metrics.AddMetrics<DeviceType>(clipMetrics);

    *LOGGER() << Logger::Info << "Mean reprojection error = " << clipMetrics.Accuracy.ReprojectionError << " pixels" << Logger::endl;
    *LOGGER() << Logger::Info << "Mean depth error = " << clipMetrics.Accuracy.DepthError * Holoportation::c_meterToMm << " mm"
        << Logger::endl;
    *LOGGER() << Logger::Info << "Multi-view misalignment = " << clipMetrics.Accuracy.MultiViewMisalignment * Holoportation::c_meterToMm
        << " mm" << Logger::endl;

    // Save calibration parameters for future reference, e.g. for recalibration, 3D reconstruction.
    SaveCalibrationParameters(devices, params.calibrationDataInput.OutDir);

    // So we have new random seed for next run.
    dataLoader.MaybeUpdateRandomSeed();

    CalibrationCompletedCallback(progressState, clipMetrics.Accuracy.ReprojectionError,
                                clipMetrics.Accuracy.DepthError * Holoportation::c_meterToMm,
                                clipMetrics.Accuracy.MultiViewMisalignment * Holoportation::c_meterToMm);

    return devices;
}

template <typename DeviceType, typename SensorInputType, typename CalibrationDataLoaderType>
void Validate(
    std::vector<DeviceType>& devices,
    const InputParameters& params,
    Metrics::ClipMetrics& clipMetrics,
    Metrics& metrics)
{
    CalibrationDataLoaderType dataLoader(params.validationDataInput);

    // Initialize calibration and FrameState objects.
    Calibration calibration(params.validationParams, devices);
    Calibration::FrameState frameState(calibration);

    calibration.SetConfig(params.configJsonPath);

    // Capture Loop:
    uint32_t frameIndex = 0;
    *LOGGER() << Logger::Info << "Validation frames start from frame " << frameIndex << " and use every " << params.valKeepFrameRate
        << "'th frame." << Logger::endl;

    while (frameIndex < params.valEndFrame)
    {
        const std::vector<SensorInputType>& sensorsData = dataLoader.GetSensorsData(frameIndex);
        if (sensorsData.empty())
            break;

        try
        {
            calibration.ProcessSensorData<SensorInputType>(sensorsData, frameState);
        }
        catch (const Holoportation::API::CalibrationDataAboveCalibrationCapacity& exception)
        {
            *LOGGER() << Logger::Error << exception.what()
                << " Will stop processing frames and compute validation stats on data processed so far."
                << Logger::endl;
            break;
        }

        if (params.validationParams.CalibrationMode == Holoportation::API::Calibration::Mode::Monitoring ||
            params.validationParams.CalibrationMode == Holoportation::API::Calibration::Mode::PassiveCalibration)
        {
            std::optional<float> monitoringScore = frameState.GetMonitoringScore();
            *LOGGER() << Logger::Info << "Monitoring score (" << frameIndex << ") = ";
            if (monitoringScore.has_value())
                *LOGGER() << *monitoringScore;
            else
                *LOGGER() << "no info";
            *LOGGER() << Logger::endl;
        }

        *LOGGER() << Logger::Info << "Processed frame " << frameIndex << "\r" << Logger::endl;
        frameIndex += params.valKeepFrameRate;

        if (params.validationParams.CalibrationMode == Holoportation::API::Calibration::Mode::PassiveCalibration &&
            frameState.IsRecalibrationNeeded())
        {
            *LOGGER() << Logger::Info << "Recalibrating..." << Logger::endl;
            try
            {
                calibration.CalibrateDevices<DeviceType>(devices, clipMetrics.Accuracy);
            }
            catch (const CalibrationCaptureIncompleteException&)
            {
                // we expect this exception since the coverage isn't complete
            }
            catch (const std::exception& e)
            {
                *LOGGER() << Logger::Error << "Calibration exception: " << e.what() << Logger::endl;
            }
            *LOGGER() << Logger::Info << "Mean reprojection error = " << clipMetrics.Accuracy.ReprojectionError << " pixels"
                << Logger::endl;
            *LOGGER() << Logger::Info << "Mean depth error = " << clipMetrics.Accuracy.DepthError * Holoportation::c_meterToMm << " mm"
                << Logger::endl;
            *LOGGER() << Logger::Info << "Multi-view misalignment = "
                << clipMetrics.Accuracy.MultiViewMisalignment * Holoportation::c_meterToMm << " mm" << Logger::endl;
        }
    }

    // Execute validation on the collected data.
    clipMetrics.Accuracy = calibration.ValidateCalibration();

    clipMetrics.RunType = Metrics::RunType::Validation;
    clipMetrics.ClipId = params.validationDataInput.MkvsDir.filename();
    clipMetrics.DeviceCount = static_cast<uint32_t>(devices.size());
    clipMetrics.NumberOfProcessedFrames = 0;
    clipMetrics.ExecutionTime = Metrics::ExecutionTime{};
    metrics.AddMetrics<DeviceType>(clipMetrics);

    if (params.validationParams.MarkerType == Holoportation::API::Calibration::MarkerType::FaceLandmarks)
        *LOGGER() << Logger::Info << "Calibration monitoring error = " << clipMetrics.Accuracy.MonitoringError << " pixels\n";
    else
        *LOGGER() << Logger::Info << "Validation Multi-view misalignment = "
        << clipMetrics.Accuracy.MultiViewMisalignment * Holoportation::c_meterToMm << " mm" << Logger::endl;
}

int RunCalibrationApi(int argc, const char** argv, 
    void* progressState,
    std::function<void(void*, uint32_t, uint32_t)> ProgressCallback,
    std::function<void(void*, float, float, float)> CalibrationCompletedCallback)
{
    try
    {
        InputParameters params;
        if (!params.ParseArguments(argc, argv, std::cerr))
        {
            params.ShowUsage(std::cerr);
            return EXIT_FAILURE;
        }

        Metrics metrics(params.metricsParams);
        Metrics::ClipMetrics clipMetrics{};
        CalibrationDataLoader dataLoaderVariant = CreateCalibrationDataLoader(params.calibrationDataInput);

        using DeviceListType = std::variant<std::vector<RgbdDevice>, std::vector<StereoIrPod>>;

        for (uint32_t runIdx = 0; runIdx < params.runCount; ++runIdx)
        {
            clipMetrics.RunIndex = runIdx;
            std::visit(
                [&](const auto& dataLoader) { clipMetrics.RandomSeed = dataLoader.GetRandomSeed(); },
                dataLoaderVariant);
            DeviceListType devices;
            if (params.calibrationParams.CalibrationMode != Holoportation::API::Calibration::Mode::Unknown)
            {
                *LOGGER() << Logger::Info << "Calibrating..." << Logger::endl;
                switch (params.calibrationDataInput.RigType)
                {
                case CalibrationDataInputParameters::RigTypeEnum::Kinect:
                    devices = Calibrate<RgbdDevice, Calibration::RgbdSensorInput, KinectCalibrationDataLoader>(
                        params, std::get<KinectCalibrationDataLoader>(dataLoaderVariant), clipMetrics, metrics,
                        progressState,
                        ProgressCallback,
                        CalibrationCompletedCallback);
                    break;
                case CalibrationDataInputParameters::RigTypeEnum::StereoIR:
                    devices =
                        Calibrate<StereoIrPod, Calibration::StereoIrPodSensorInput, StereoIRCalibrationDataLoader>(
                            params, std::get<StereoIRCalibrationDataLoader>(dataLoaderVariant), clipMetrics, metrics,
                            progressState,
                            ProgressCallback,
                            CalibrationCompletedCallback);
                    break;
                }
            }
            else
            {
                std::visit(
                    [&](const auto& dataLoader) { devices = dataLoader.GetDeviceCalibrations(); }, dataLoaderVariant);
            }

            if (!params.validationDataInput.MkvsDir.empty())
            {
                *LOGGER() << Logger::Info << "Validating..." << Logger::endl;
                switch (params.calibrationDataInput.RigType)
                {
                case CalibrationDataInputParameters::RigTypeEnum::Kinect: {
                    std::vector<RgbdDevice>& rgbdDevices = std::get<std::vector<RgbdDevice>>(devices);
                    params.validationDataInput.NumberOfDevices = static_cast<uint32_t>(rgbdDevices.size());
                    Validate<RgbdDevice, Calibration::RgbdSensorInput, KinectCalibrationDataLoader>(
                        rgbdDevices, params, clipMetrics, metrics);
                    break;
                }
                case CalibrationDataInputParameters::RigTypeEnum::StereoIR: {
                    std::vector<StereoIrPod>& stereoIrPodDevices = std::get<std::vector<StereoIrPod>>(devices);
                    params.validationDataInput.NumberOfDevices = static_cast<uint32_t>(stereoIrPodDevices.size());
                    Validate<StereoIrPod, Calibration::StereoIrPodSensorInput, StereoIRCalibrationDataLoader>(
                        stereoIrPodDevices, params, clipMetrics, metrics);
                    break;
                }
                }
            }
        }
    }
    catch (std::exception& ex)
    {
        *LOGGER() << Logger::Error << ex.what() << Logger::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}