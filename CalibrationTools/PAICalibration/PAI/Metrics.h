/**
* File copied from the RunCalibrationApi executable source code of the PAI Calibration tool.
* 
* Repo: https://microsoft.visualstudio.com/Analog/_git/vision.presence.babylon?path=/Holoportation/Tools/Calibration/RunCalibrationApi
* Commit: f7e69cc2
**/

#pragma once

#include <Calibration/CalibrationAPI.h>
#include <Common/ApTableSentinelValues.h>
#include <Common/Conversions.h>
#include <Common/DeviceProperties.h>
#include <Common/SafeFileStream.h>

#include <ts/matrix.h>
#include <ts/mean.h>

#include <numeric>
#include <optional>
#include <vector>

class Metrics
{
public:
    struct Params
    {
        std::filesystem::path ClipMetricsPath;
        std::filesystem::path DetailsMetricsPath;
    };

    enum class RunType
    {
        Unknown = 0,
        Train = 1,
        Validation = 2,
        Test = 3,
    };

    struct ExecutionTime
    {
        float PreprocessingPerFrame = ApNotApplicable;
        float Optimization = ApNotApplicable;
        float Total = ApNotApplicable;
    };

    struct FloorMetric
    {
        float NormalsAngle;
        float XAxesAngle;
        float ZAxesAngle;
        float YTranslation;
        float CentersDistance;
    };

    struct ClipMetrics
    {
        RunType RunType;
        std::wstring ClipId;
        Holoportation::API::Calibration::Accuracy Accuracy;
        ExecutionTime ExecutionTime;
        uint32_t RunIndex;
        uint16_t RandomSeed;
        uint32_t DeviceCount;
        uint32_t NumberOfProcessedFrames;
        std::optional<FloorMetric> ClipFloorMetric;
    };

    Metrics(const Params& params);

    template <typename DeviceType>
    void AddMetrics(const ClipMetrics& clipMetrics)
    {
        using namespace Holoportation::API;
        if (m_clipMetricsFile.good())
        {
            float reprojectionError =
                clipMetrics.RunType == RunType::Train ? clipMetrics.Accuracy.ReprojectionError : ApNotApplicable;
            float depthError =
                clipMetrics.RunType == RunType::Train ? clipMetrics.Accuracy.DepthError : ApNotApplicable;
            float depthToColorReprojection = clipMetrics.Accuracy.DepthToColorReprojectionError.size() > 0
                ? std::reduce(
                    clipMetrics.Accuracy.DepthToColorReprojectionError.begin(),
                    clipMetrics.Accuracy.DepthToColorReprojectionError.end()) /
                clipMetrics.Accuracy.DepthToColorReprojectionError.size()
                : ApNotApplicable;

            // Write a row into the table.
            // clang-format off
            m_clipMetricsFile
                << RunTypeToString(clipMetrics.RunType) << ","
                << clipMetrics.ClipId << ","
                << clipMetrics.RunIndex << ","
                << clipMetrics.RandomSeed << ","
                << reprojectionError << ","
                << depthError << ","
                << clipMetrics.Accuracy.MultiViewMisalignment << ","
                << depthToColorReprojection << ","
                << clipMetrics.ExecutionTime.PreprocessingPerFrame << ","
                << clipMetrics.ExecutionTime.Optimization << ","
                << clipMetrics.ExecutionTime.Total << ","
                << clipMetrics.NumberOfProcessedFrames << ","
                << (clipMetrics.ClipFloorMetric ? clipMetrics.ClipFloorMetric->NormalsAngle : ApNotApplicable) << ","
                << (clipMetrics.ClipFloorMetric ? clipMetrics.ClipFloorMetric->XAxesAngle : ApNotApplicable) << ","
                << (clipMetrics.ClipFloorMetric ? clipMetrics.ClipFloorMetric->ZAxesAngle : ApNotApplicable) << ","
                << (clipMetrics.ClipFloorMetric ? clipMetrics.ClipFloorMetric->YTranslation : ApNotApplicable) << ","
                << (clipMetrics.ClipFloorMetric ? clipMetrics.ClipFloorMetric->CentersDistance : ApNotApplicable)
                << std::endl;
            // clang-format on
        }

        if (m_detailsMetricsFile.good())
        {
            // Write rows into the table.
            const auto framesCount = clipMetrics.Accuracy.Details.GetNumberOfFrames();
            const auto fiducialsCount = clipMetrics.Accuracy.Details.GetNumberOfFiducials();
            const std::vector<Subcamera> subcameras = DeviceProperties::GetAvailableSubcameras<DeviceType>();
            for (uint32_t deviceIdx = 0; deviceIdx < clipMetrics.DeviceCount; ++deviceIdx)
                for (Holoportation::API::Subcamera subcamera : subcameras)
                    for (uint32_t frameIdx = 0; frameIdx < framesCount; ++frameIdx)
                    {
                        const bool isFrameRejected =
                            clipMetrics.Accuracy.Details.IsFrameRejected<DeviceType>(deviceIdx, subcamera, frameIdx);
                        for (uint32_t fiducialIdx = 0; fiducialIdx < fiducialsCount; ++fiducialIdx)
                        {
                            auto optionalDetectionUv = clipMetrics.Accuracy.Details.GetDetectedFiducial<DeviceType>(
                                deviceIdx, subcamera, frameIdx, fiducialIdx);
                            auto detectionUv = optionalDetectionUv ? *optionalDetectionUv : ApNotApplicable2d;
                            auto optionalCorrectedZ = clipMetrics.Accuracy.Details.GetFiducialCorrectedZ<DeviceType>(
                                deviceIdx, subcamera, frameIdx, fiducialIdx);
                            auto correctedZ = optionalCorrectedZ ? *optionalCorrectedZ : ApNotApplicable;
                            auto optionalReprojectionUv =
                                clipMetrics.Accuracy.Details.GetReprojectedFiducial<DeviceType>(
                                    deviceIdx, subcamera, frameIdx, fiducialIdx);
                            auto reprojectionUv = optionalReprojectionUv ? *optionalReprojectionUv : ApNotApplicable2d;
                            auto optionalOptimizedView = clipMetrics.Accuracy.Details.GetOptimizedViewPoint<DeviceType>(
                                deviceIdx, subcamera, frameIdx, fiducialIdx);
                            auto optimizedView = optionalOptimizedView ? *optionalOptimizedView : ApNotApplicable3d;

                            auto unprojectedWorld = ApNotApplicable3d;

                            auto optionalOptimizedRayIntersectionReprojectionError =
                                clipMetrics.Accuracy.Details.GetOptimizedRayIntersectionReprojectionError<DeviceType>(
                                    deviceIdx, subcamera, frameIdx, fiducialIdx);
                            auto optimizedRayIntersectionReprojectionError =
                                optionalOptimizedRayIntersectionReprojectionError
                                ? *optionalOptimizedRayIntersectionReprojectionError
                                : ApNotApplicable;

                            // Find the average position (barycenter) of the depth camera fiducial unprojected to the
                            // world.
                            ts::mean<Vec3f> barycenter{ Vec3f{0, 0, 0} };
                            for (uint32_t deviceForBarycenterIdx = 0; deviceForBarycenterIdx < clipMetrics.DeviceCount;
                                ++deviceForBarycenterIdx)
                            {
                                if (const auto& worldPoint =
                                    clipMetrics.Accuracy.Details.GetUnprojectedWorldPoint<DeviceType>(
                                        deviceForBarycenterIdx, subcamera, frameIdx, fiducialIdx))
                                {
                                    barycenter += *worldPoint;
                                    if (deviceForBarycenterIdx == deviceIdx)
                                        unprojectedWorld = *worldPoint;
                                }
                            }

                            // If more than one view observed the same fiducial, find the deviation of current view
                            // fiducial in 3D compared to the barycenter.
                            auto dWorld = ApNotApplicable3d;
                            if (barycenter.count() > 1)
                                dWorld = unprojectedWorld - barycenter.mu();

                            // clang-format off
                            const std::string subcameraString = Holoportation::SubcameraToString(subcamera);
                            m_detailsMetricsFile
                                << RunTypeToString(clipMetrics.RunType) << ","
                                << clipMetrics.ClipId << ","
                                << clipMetrics.RunIndex << ","
                                << clipMetrics.RandomSeed << ","
                                << deviceIdx << ","
                                << std::wstring{ subcameraString.begin(), subcameraString.end() } << ","
                                << frameIdx << ","
                                << isFrameRejected << ","
                                << fiducialIdx << ","
                                << optionalDetectionUv.has_value() << ","
                                << detectionUv[0] << ","
                                << detectionUv[1] << ","
                                << correctedZ << ","
                                << unprojectedWorld[0] << ","
                                << unprojectedWorld[1] << ","
                                << unprojectedWorld[2] << ","
                                << dWorld[0] << ","
                                << dWorld[1] << ","
                                << dWorld[2] << ","
                                << (dWorld == ApNotApplicable3d ? ApNotApplicable : ts::length(dWorld)) << ","
                                << barycenter.count() << ","
                                << optionalReprojectionUv.has_value() << ","
                                << reprojectionUv[0] << ","
                                << reprojectionUv[1] << ","
                                << optimizedView[0] << ","
                                << optimizedView[1] << ","
                                << optimizedView[2] << ","
                                << optimizedRayIntersectionReprojectionError << '\n';
                            // clang-format on
                        }
                    }
        }
    }

    template <typename DeviceType>
    static FloorMetric ComputeClipFloorMetric(const ts::matrix4x4f& annotatedWorldToDepth, const DeviceType& device);
    template <>
    static FloorMetric ComputeClipFloorMetric(
        const ts::matrix4x4f& annotatedWorldToDepth, const Holoportation::API::RgbdDevice& device);
    template <>
    static FloorMetric ComputeClipFloorMetric(
        const ts::matrix4x4f& annotatedWorldToDepth, const Holoportation::API::StereoIrPod& device);

private:
    SafeFileStream m_clipMetricsFile;
    SafeFileStream m_detailsMetricsFile;

    using TableFormat = std::vector<std::pair<std::wstring_view, std::wstring_view>>;

    static void InitializeOutputMetricsFile(
        SafeFileStream& file, const std::filesystem::path& path, const TableFormat& tableFormat);

    static void WriteHeaderAndSchema(
        SafeFileStream& resultsFile, std::wofstream& schemaFile, const TableFormat& tableFormat);

    static std::wstring RunTypeToString(RunType runType);

    static constexpr std::string_view c_schemaSuffix = ".schema.md";

    static inline const TableFormat ClipTableFormat{
        {L"RunType", L"string"},
        {L"ClipId", L"string"},
        {L"RunIdx", L"int"},
        {L"RandomSeed", L"int"},
        {L"ReprojectionError", L"float"},
        {L"DepthError", L"float"},
        {L"MultiViewMisalignment", L"float"},
        {L"DepthToColorReprojection", L"float"},
        {L"ExecutionTime.PreprocessingPerFrame", L"float"},
        {L"ExecutionTime.Optimization", L"float"},
        {L"ExecutionTime.Total", L"float"},
        {L"NumberOfProcessedFrames", L"int"},
        {L"FloorNormalsAngle", L"float"},
        {L"FloorXAxesAngle", L"float"},
        {L"FloorZAxesAngle", L"float"},
        {L"FloorYTranslation", L"float"},
        {L"FloorCentersDistance", L"float"},
    };

    static inline const TableFormat DetailsTableFormat{
        {L"RunType", L"string"},
        {L"ClipId", L"string"},
        {L"RunIdx", L"int"},
        {L"RandomSeed", L"int"},
        {L"DeviceIdx", L"int"},
        {L"CameraType", L"string"},
        {L"FrameIdx", L"int"},
        {L"IsFrameRejected", L"int"},
        {L"FiducialId", L"int"},
        {L"IsDetected", L"int"},
        {L"Detection.u", L"float"},
        {L"Detection.v", L"float"},
        {L"CorrectedZ", L"float"},
        {L"UnprojectedWorld.x", L"float"},
        {L"UnprojectedWorld.y", L"float"},
        {L"UnprojectedWorld.z", L"float"},
        {L"dWorld.x", L"float"},
        {L"dWorld.y", L"float"},
        {L"dWorld.z", L"float"},
        {L"dWorld.d", L"float"},
        {L"MultiViewPointsCount", L"int"},
        {L"IsReprojected", L"int"},
        {L"Reprojection.u", L"float"},
        {L"Reprojection.v", L"float"},
        {L"OptimizedView.x", L"float"},
        {L"OptimizedView.y", L"float"},
        {L"OptimizedView.z", L"float"},
        {L"OptimizedRayIntersectionReprojectionError", L"float"},
    };
};