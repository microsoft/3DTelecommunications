/**
* File copied from the RunCalibrationApi executable source code of the PAI Calibration tool.
* Repo: https://microsoft.visualstudio.com/Analog/_git/vision.presence.babylon?path=/Holoportation/Tools/Calibration/RunCalibrationApi
* Commit: f7e69cc2
**/

#pragma once

#include "Metrics.h"

#include <Calibration/CalibrationAPI.h>
#include <Common/CalibrationDataLoader.h>
#include <Common/CommonUtils.h>
#include <Common/Visualization.h>

#include <Useful/Arguments.h>

// Partial representation of the contents of the JSON metadata files.
struct RunConfiguration
{
    float CalibBoardSizeMm;
    std::string CalibMarkerType;
};
inline void Read(const Useful::Json::Value& value, RunConfiguration& runConfiguration)
{
    Read(value["calib_board_size_mm"], runConfiguration.CalibBoardSizeMm);
    Read(value["calib_marker_type"], runConfiguration.CalibMarkerType);
}

struct InputParameters
{
public:
    InputParameters()
    {
        parser.AddRequiredOption(rigTypeString, "--rigType", "Kinect / StereoIR");
        parser.AddRequiredOption(calibrationModeString, "--calibration-mode", "none / stage1 / stage2");
        parser.AddOption(
            calibrationDataInput.MkvsDir,
            "--mkvsDir",
            "Required for Kinect rigs. Path to directory containing MKV per AK device. Data used from MKV includes "
            "cameras' frames, factory calibration, IMU samples, parameters of objects used for calibration stored in "
            "objectParameters.json (if not present object size must be provided. Default 6x8 Nova Board with provided "
            "size will be used).");
        parser.AddRequiredOption(
            calibrationDataInput.OutDir, "--outDir", "Path to directory where output files should be saved.");
        parser.AddOption(
            calibrationDataInput.FramesDir,
            "--framesDir",
            "Path to directory containing a directory per camera, where each camera dir contains images from that "
            "camera. If provided, it will take precedence over frames from MKVs.");
        parser.AddOption(
            calibrationDataInput.IsIr8Bit,
            "--irIs8Bit",
            "Flag denoting that frames data has 8 bit IR. This is deprecated MW format.");
        parser.AddOption(
            calibrationDataInput.MwParamsDir,
            "--mwParamsDir",
            "Path to directory containing camExtrin.txt files which were created by MW calibration tool. If provided, "
            "it will take precendence over calibration parameters from MKVs.");
        parser.AddOption(
            calibrationDataInput.DevicesCalibrationJson,
            "--devicesCalibrationJson",
            "Path to the devicesCalibration.json. If provided, it will take precedence over calibration parameters "
            "from MKVs or MW params files.");
        parser.AddOption(
            calibrationDataInput.NumberOfDevices,
            "--numOfDevices",
            "Number of devices to be calibrated. Default is 3.");
        parser.AddOption(
            calibrationParams.MaximumTimeSeparationOfSubframes,
            "--maximumTimeSeparationOfSubframes",
            "Value is in microseconds. For a given set of camera inputs passed to ProcessSensorData(), it specifies "
            "the maximal timestamp difference allowed. The smaller the value, the more accurate calibration will be. A "
            "larger value may be necessary for a capture system with larger number of cameras which need to be offset "
            "in time to avoid interference.");
        parser.AddOption(
            calibrationDataInput.RandomSeed,
            "--randomSeed",
            "Unsigned 16bit integer to be used as with clip GUID as random seed");
        parser.AddOption(
            calibrationDataInput.EmptyClipPathWithAnnotatedFloor,
            "--emptyClipPathWithAnnotatedFloor",
            "Path to empty clip with annotated floor. In the annotated floor directory we need depth frame and also we "
            "find middle AK index from directory name output-deviceIndex-extracted");
        parser.AddOption(
            calibrationDataInput.SequenceID, "--sequenceID", "Sequence identifier (required for stereo rigs).");
        parser.AddOption(
            calibrationParams.CalibrationObjectSizeInMeters,
            "--objectSize",
            "Characteristic size of the calibration object (e.g. length of one square) in meters.");
        parser.AddOption(
            markerTypeString,
            "--markerType",
            "Type of the marker present in the calibration board or scene (Aruco/Nova/FaceLandmarks). Default is "
            "Nova.");
        parser.AddOption(
            calibrationParams.SourceForWorldTransforms.UseImu,
            "--useImuToAlignYAxisToGravity",
            "If passed, obtained calibration parameters will be given in world CS so that y-axis is aligned with the "
            "average Gravity vector.");
        parser.AddOption(
            calibrationParams.SourceForWorldTransforms.UseDetectedFloor,
            "--useDetectedFloorForWorldAlignment",
            "If passed, and floor is successfuly detected, obtained calibration parameters will be given in world CS "
            "so that floor plane is in y=0. Has precedence over --useImuToAlignYAxisToGravity.");
        parser.AddOption(validationModeString, "--validation-mode", "live / offline / passive. Default is offline.");
        parser.AddOption(
            validationMarkerTypeString,
            "--valMarkerType",
            "Type of the marker present in the calibration board or scene (Aruco/Nova/FaceLandmarks). Default is "
            "Nova.");
        parser.AddOption(
            validationDataInput.MkvsDir,
            "--valDir",
            "Path to directory containing MKV per AK device and parameters of objects used for validation stored in "
            "objectParameters.json (if not present object size must be provided. Default 6x8 Nova Board with provided "
            "size will be used). This data will be used to validate the quality of previously obtained calibration.");
        parser.AddOption(
            validationDataInput.FramesDir,
            "--valFramesDir",
            "Path to directory containing directory per camera dir contains images from that camera. If provided, it "
            "will take precedence over frames from MKVs.");
        parser.AddOption(
            validationParams.CalibrationObjectSizeInMeters,
            "--valObjectSize",
            "Characteristic size of the calibration object (e.g. length of one square) in meters for validation.");
        parser.AddOption(startFrame, "--startFrame", "First frame in the sequence to process.");
        parser.AddOption(
            endFrame,
            "--endFrame",
            "Last frame in the sequence to process. If not set use calibration IsDataCaptureComplete to stop feeding "
            "data.");
        parser.AddOption(keepFrameRate, "--keepFrameRate", "Will process only every `keepFrameRate`'th frame.");
        parser.AddOption(
            valKeepFrameRate, "--valKeepFrameRate", "Will process only every `keepFrameRate`'th frame in validation.");
        parser.AddOption(
            valEndFrame,
            "--valEndFrame",
            "Last frame in the validation sequence to process. If not set use calibration IsDataCaptureComplete to "
            "stop feeding data.");
        parser.AddOption(runCount, "--runCount", "Number of times to repeat calibration on input data.");
        parser.AddOption(
            metadataPath,
            "--metadata",
            "JSON file with metadata for the clip. Values may be overridden by other flags.");
        parser.AddOption(
            validationMetadataPath,
            "--valMetadata",
            "JSON file with metadata for the validation clip. Values may be overridden by other flags.");
        parser.AddRequiredOption(
            metricsParams.ClipMetricsPath,
            "--clipMetricsFilePath",
            "Path to the csv file metrics results will be saved to.");
        parser.AddOption(
            metricsParams.DetailsMetricsPath,
            "--detailsMetricsFilePath",
            "Path to the csv file containing detailed logs for per-fiducial metrics.");
        parser.AddOption(
            configJsonPath,
            "--configJsonPath",
            "Path to JSON configuring calibration internal state. For debugging purpose only!");
        parser.AddOption(
            doNotThrowOnMissingData, "--doNotThrowOnMissingData", "Do not throw on missing data in data loader.");
        parser.AddOption(
            visualizationParams.SaveFiducialDetectionImages,
            "--saveFiducialDetectionImages",
            "Visualizes all the detected fiducial markers in camera frame.");
        parser.AddOption(
            visualizationParams.SaveFiducialImages,
            "--saveFiducialImages",
            "Visualizes all the detected and reprojected fiducial markers in camera frame.");
        parser.AddOption(
            visualizationParams.SaveFovCoverage,
            "--saveFovCoverage",
            "Visualizes the cumulative fiducial markers in camera frame.");
        parser.AddOption(
            visualizationParams.SaveCoordinateSystems,
            "--saveCoordinateSystems",
            "Visualize world coordinate system as mesh.");
        parser.AddOption(
            visualizationParams.SaveFloorEquations,
            "--saveFloorEquations",
            "Log floor equations in Hesse normal form.");

        // Set defaults.
        calibrationDataInput.NumberOfDevices = 3;
        calibrationParams.MarkerType = Holoportation::API::Calibration::MarkerType::Nova;
        calibrationParams.CalibrationObjectSizeInMeters = 0.0f;      // (set to an invalid value to now)
        calibrationParams.SourceForWorldTransforms = { false, false }; // Default to none. Use cmd flags for alignment.
        validationParams.MarkerType = Holoportation::API::Calibration::MarkerType::Nova;
        validationParams.CalibrationMode = Holoportation::API::Calibration::Mode::Validation;
        validationParams.CalibrationObjectSizeInMeters = 0.0f; // (set to an invalid value to now)
    }

    bool ParseArguments(int argc, const char* argv[], std::ostream& outputStream)
    {
        return (parser.ParseArguments(argc, argv) && ValidateSettings(outputStream));
    }

    void ShowUsage(std::ostream& outputStream) const
    {
        parser.ShowUsage(outputStream);
    }

    static constexpr uint32_t c_endFrameNotSet = std::numeric_limits<uint32_t>::max();
    Holoportation::API::Calibration::Params calibrationParams;
    Holoportation::API::Calibration::Params validationParams;
    CalibrationDataInputParameters calibrationDataInput;
    CalibrationDataInputParameters validationDataInput;
    Holoportation::Visualization::CalibrationVisualizer::Params visualizationParams;
    Metrics::Params metricsParams;
    uint32_t startFrame = 0;
    uint32_t endFrame = c_endFrameNotSet;
    uint32_t keepFrameRate = 1;
    uint32_t valKeepFrameRate = 1;
    uint32_t valEndFrame = c_endFrameNotSet;
    uint32_t runCount = 1;
    std::filesystem::path metadataPath;
    std::filesystem::path validationMetadataPath;
    std::filesystem::path configJsonPath;
    bool doNotThrowOnMissingData = false;

private:
    bool ValidateSettings(std::ostream& outputStream)
    {
        if (doNotThrowOnMissingData)
        {
            if (!validationDataInput.MkvsDir.empty() && GetDirectoryItemCount(validationDataInput.MkvsDir) == 0)
            {
                outputStream << "Warning: directory \"" << validationDataInput.MkvsDir.string()
                    << "\" is empty and it will not be used." << std::endl;
                validationDataInput.MkvsDir.clear();
            }
            if (!calibrationDataInput.EmptyClipPathWithAnnotatedFloor.empty() &&
                GetDirectoryItemCount(calibrationDataInput.EmptyClipPathWithAnnotatedFloor) == 0)
            {
                outputStream << "Warning: directory \"" << calibrationDataInput.EmptyClipPathWithAnnotatedFloor.string()
                    << "\" is empty and it will not be used." << std::endl;
                calibrationDataInput.EmptyClipPathWithAnnotatedFloor.clear();
            }
            if (!validationMetadataPath.empty() && !std::filesystem::exists(validationMetadataPath))
            {
                outputStream << "Warning: file \"" << validationMetadataPath.string()
                    << "\" does not exist and it will not be used." << std::endl;
                validationMetadataPath.clear();
            }
        }

        // Read clip metadata file.
        if (!SetFromMetadataOrDefault(
            metadataPath,
            outputStream,
            calibrationParams.MarkerType,
            calibrationParams.CalibrationObjectSizeInMeters))
            return false;

        // Read validation clip metadata file.
        if (!SetFromMetadataOrDefault(
            validationMetadataPath,
            outputStream,
            validationParams.MarkerType,
            validationParams.CalibrationObjectSizeInMeters))
            return false;

        // Interpret the rig type.
        if (rigTypeString == "Kinect")
            calibrationDataInput.RigType = CalibrationDataInputParameters::RigTypeEnum::Kinect;
        else if (rigTypeString == "StereoIR")
            calibrationDataInput.RigType = CalibrationDataInputParameters::RigTypeEnum::StereoIR;
        else
        {
            outputStream << "Unknown rig type specified: " << rigTypeString << std::endl;
            return false;
        }

        if (calibrationDataInput.RigType == CalibrationDataInputParameters::RigTypeEnum::Kinect &&
            calibrationDataInput.MkvsDir.empty())
        {
            outputStream << "--mkvsDir option is required for Kinect rigs" << std::endl;
            return false;
        }
        if (calibrationDataInput.RigType == CalibrationDataInputParameters::RigTypeEnum::StereoIR &&
            calibrationDataInput.SequenceID.empty())
        {
            outputStream << "--sequenceID option is required for stereo IR rigs" << std::endl;
            return false;
        }

        // Check if provided paths are valid.
        if (calibrationDataInput.RigType == CalibrationDataInputParameters::RigTypeEnum::Kinect &&
            !std::filesystem::exists(calibrationDataInput.MkvsDir))
        {
            outputStream << "MKVs directory does not exist." << std::endl;
            return false;
        }
        if (metricsParams.ClipMetricsPath.empty())
        {
            outputStream << "MetricsFilePath needs to be specified." << std::endl;
            return false;
        }
        if (!calibrationDataInput.FramesDir.empty() && !std::filesystem::exists(calibrationDataInput.FramesDir))
        {
            outputStream << "Frames directory does not exist - " << calibrationDataInput.FramesDir << std::endl;
            return false;
        }
        if (!calibrationDataInput.MwParamsDir.empty() && !std::filesystem::exists(calibrationDataInput.MwParamsDir))
        {
            outputStream << "MW params directory does not exist - " << calibrationDataInput.MwParamsDir << std::endl;
            return false;
        }
        if (!calibrationDataInput.DevicesCalibrationJson.empty() &&
            !std::filesystem::exists(calibrationDataInput.DevicesCalibrationJson))
        {
            outputStream << "File does not exist - " << calibrationDataInput.DevicesCalibrationJson << std::endl;
            return false;
        }

        // Interpret the calibration mode.
        if (calibrationModeString == "none")
            calibrationParams.CalibrationMode = Holoportation::API::Calibration::Mode::Unknown;
        else if (calibrationModeString == "stage1")
            calibrationParams.CalibrationMode = Holoportation::API::Calibration::Mode::IntrinsicAndExtrinsicCalibration;
        else if (calibrationModeString == "stage2")
            calibrationParams.CalibrationMode = Holoportation::API::Calibration::Mode::ExtrinsicCalibration;
        else
        {
            outputStream << "Unknown calibration mode specified: " << calibrationModeString << std::endl;
            return false;
        }

        // parse the marker type strings (will overwrite values from metadata if supplied)
        ParseMarkerTypeIfSupplied(markerTypeString, calibrationParams.MarkerType);
        ParseMarkerTypeIfSupplied(validationMarkerTypeString, validationParams.MarkerType);

        // Set calibration objects, if provided
        if (!SetObjects(calibrationDataInput, calibrationParams, outputStream))
            return false;

        // Set validation objects
        if (!SetObjects(validationDataInput, validationParams, outputStream))
            return false;

        // Interpret the validation mode
        if (validationModeString == "live")
            validationParams.CalibrationMode = Holoportation::API::Calibration::Mode::Monitoring;
        else if (validationModeString == "offline")
            validationParams.CalibrationMode = Holoportation::API::Calibration::Mode::Validation;
        else if (validationModeString == "passive")
            validationParams.CalibrationMode = Holoportation::API::Calibration::Mode::PassiveCalibration;

        if (!(keepFrameRate >= 1))
        {
            outputStream << "`keepFrameRate` needs to be a non-zero positive integer." << std::endl;
            return false;
        }

        if (!(valKeepFrameRate >= 1))
        {
            outputStream << "`valKeepFrameRate` needs to be a non-zero positive integer." << std::endl;
            return false;
        }

        return true;
    }

    bool SetObjects(
        CalibrationDataInputParameters& dataInput,
        Holoportation::API::Calibration::Params& params,
        std::ostream& outputStream)
    {
        const auto objectsPath = dataInput.MkvsDir.string() + "/objectParameters.json";
        if (std::filesystem::exists(objectsPath))
        {
            auto calibrationObjects =
                Read<std::vector<Holoportation::DetectableObject>>(objectsPath, "objectParameters");
            for (auto& obj : calibrationObjects)
            {
                if (static_cast<Holoportation::API::Calibration::MarkerType>(obj.GetMarkerType()) != params.MarkerType)
                {
                    outputStream << "Incorrect marker ID on at least one object.\n";
                    return false;
                }
            }

            for (auto& obj : calibrationObjects)
            {
                params.MarkerPointsPerObject.push_back(obj.GetMarkerPoints());
                params.FiducialIdsPerObject.push_back(obj.GetFiducialIds());
            }
        }
        else if ((dataInput.MkvsDir != "" || dataInput.FramesDir != "") && params.CalibrationObjectSizeInMeters <= 0)
        {
            outputStream << "Neither valid object size nor objectParameters.json provided.\n";
            return false;
        }
        return true;
    }

    void ParseMarkerTypeIfSupplied(
        const std::string& markerTypeString, Holoportation::API::Calibration::MarkerType& markerType)
    {
        if (markerTypeString == "Aruco")
            markerType = Holoportation::API::Calibration::MarkerType::Aruco;
        else if (markerTypeString == "Nova")
            markerType = Holoportation::API::Calibration::MarkerType::Nova;
        else if (markerTypeString == "FaceLandmarks")
            markerType = Holoportation::API::Calibration::MarkerType::FaceLandmarks;
    }

    bool SetFromMetadataOrDefault(
        const std::filesystem::path& metadataPath,
        std::ostream& outputStream,
        Holoportation::API::Calibration::MarkerType& markerType,
        float& objectSizeInMeters)
    {
        if (!metadataPath.empty())
        {
            if (!std::filesystem::exists(metadataPath))
            {
                outputStream << "Metadata file does not exist." << std::endl;
                return false;
            }
            const RunConfiguration runConfiguration = Read<RunConfiguration>(metadataPath, "run_configuration");
            ParseMarkerTypeIfSupplied(runConfiguration.CalibMarkerType, markerType); // (may be overwritten later)
            if (objectSizeInMeters <= 0)
            {
                // object size was not set, use the value from the metadata
                objectSizeInMeters = runConfiguration.CalibBoardSizeMm * Holoportation::c_mmToMeter;
            }
        }
        return true;
    }

    Useful::Arguments::Parser parser;
    std::string rigTypeString;
    std::string calibrationModeString;
    std::string markerTypeString;
    std::string validationMarkerTypeString;
    std::string validationModeString;
};
