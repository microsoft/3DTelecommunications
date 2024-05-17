/**
* File copied from the RunCalibrationApi executable source code of the PAI Calibration tool.
* 
* Repo: https://microsoft.visualstudio.com/Analog/_git/vision.presence.babylon?path=/Holoportation/Tools/Calibration/RunCalibrationApi
* Commit: f7e69cc2
**/

#include "Metrics.h"

#include <ts/vec.h>

#include <cmath>
#include <vector>

Metrics::Metrics(const Params& params)
    : m_clipMetricsFile(params.ClipMetricsPath), m_detailsMetricsFile(params.DetailsMetricsPath)
{
    InitializeOutputMetricsFile(m_clipMetricsFile, params.ClipMetricsPath, ClipTableFormat);
    InitializeOutputMetricsFile(m_detailsMetricsFile, params.DetailsMetricsPath, DetailsTableFormat);
}

void Metrics::InitializeOutputMetricsFile(
    SafeFileStream& file, const std::filesystem::path& path, const TableFormat& tableFormat)
{
    if (file.good())
    {
        file.precision(8);

        auto schemaPath = path;
        schemaPath.replace_extension(c_schemaSuffix);
        std::wofstream schemaFile(schemaPath);
        WriteHeaderAndSchema(file, schemaFile, tableFormat);
    }
}

void Metrics::WriteHeaderAndSchema(
    SafeFileStream& resultsFile, std::wofstream& schemaFile, const TableFormat& tableFormat)
{
    // Write header and schema for mandatory outputs.
    for (const auto& column : tableFormat)
    {
        const auto& columnName = column.first;
        const auto& columnType = column.second;

        // Write to header.
        resultsFile << columnName;
        if (column != tableFormat.back())
            resultsFile << ",";

        // Write to schema.
        schemaFile << "## " << columnName << std::endl << "`" << columnType << "`" << std::endl << std::endl;
    }

    resultsFile << std::endl;
}

std::wstring Metrics::RunTypeToString(RunType runType)
{
    switch (runType)
    {
    case RunType::Train:
        return L"Train";
    case RunType::Validation:
        return L"Validation";
    case RunType::Test:
        return L"Test";
    default:
        return L"Unknown";
    }
}

// Static.
template <>
Metrics::FloorMetric Metrics::ComputeClipFloorMetric(
    const ts::matrix4x4f& annotatedWorldToDepth, const Holoportation::API::RgbdDevice& device)
{
    using namespace Holoportation::API;

    if (!device.WorldToDevice)
        throw std::exception("World to device is not set.");

    // Get world to depth.
    const auto calibWorldToDevice = Holoportation::FromAPI(*device.WorldToDevice);
    const auto deviceToDepth = Holoportation::FromAPI(device.DepthCamera.parentToView);
    const auto calibWorldToDepth = deviceToDepth * calibWorldToDevice;
    Metrics::FloorMetric floorMetric{};
    // x, y and z axis of a coordinate system.
    std::array<Vec3f, 3> annotatedAxes;
    std::array<Vec3f, 3> calibAxes;
    for (size_t columnIdx = 0; columnIdx < 3; ++columnIdx)
    {
        annotatedAxes[columnIdx] = ts::column<3>(annotatedWorldToDepth, columnIdx).array();
        calibAxes[columnIdx] = ts::column<3>(calibWorldToDepth, columnIdx).array();
    }
    const Vec3f annotatedCenter = ts::column<3>(annotatedWorldToDepth, 3).array();
    const Vec3f calibCenter = ts::column<3>(calibWorldToDepth, 3).array();
    // For vectors a and b we use formula dot(a, b) = |a||b|cos(a, b) to find angle between vectors.
    // As axes vectors are unit we use acos of their dot product to get angle between axes.
    floorMetric.NormalsAngle = std::acos(ts::dot(annotatedAxes[1], calibAxes[1]));
    floorMetric.XAxesAngle = std::acos(ts::dot(annotatedAxes[0], calibAxes[0]));
    floorMetric.ZAxesAngle = std::acos(ts::dot(annotatedAxes[2], calibAxes[2]));
    floorMetric.CentersDistance = ts::distance(annotatedCenter, calibCenter);

    // Transform finding intersection of annotated y-axis with calibration floor plane (i.e. y-translation)
    // to annotated world coordinate system.
    const auto calibToAnnotated = ts::inverse_rigid(annotatedWorldToDepth) * calibWorldToDepth;
    const auto calibFloorCenter = ts::column<3>(calibToAnnotated, 3).array();
    const auto calibFloorNormal = ts::column<3>(calibToAnnotated, 1).array(); // I.e. y-axis.
    if (std::abs(calibFloorNormal[1]) < std::numeric_limits<float>::epsilon())
        throw std::exception("Y translation cannot be calculated.");
    floorMetric.YTranslation = ts::dot(calibFloorCenter, calibFloorNormal) / calibFloorNormal[1];

    return floorMetric;
}

// Static.
template <>
Metrics::FloorMetric Metrics::ComputeClipFloorMetric(
    const ts::matrix4x4f& annotatedWorldToDepth, const Holoportation::API::StereoIrPod& device)
{
    // TODO 40881959: Floor fitting for stereo IR data
    return {};
}
