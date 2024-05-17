#pragma once

#include "K4ACommon.h"
#include "K4ACaptureSource.h"

namespace K4AToFusionUtils
{   

    bool CreateFolder(const char* path);

    void IgnoreComments(std::ifstream& ifs);

    void GetDateStr(char* outBuf, size_t bufLen);

    void ShowImage(const char* windowName, uint32_t height, uint32_t width, uint16_t cvType, unsigned char* data, int w, int x = 0, int y = 0, double scale = 1.0);
    void ShowImageColorScale(const char* windowName, uint32_t height, uint32_t width, uint16_t cvType, unsigned char* data, int w, int x, int y, double scale = 1.0);

    template <class buff_t>
    bool LoadBufferFromBin(const char* fileName, uint32_t height, uint32_t width, buff_t* data);

    template <class buff_t>
    bool SaveBufferToBin(const char *fileName, uint32_t height, uint32_t width, buff_t *data);

    void SaveDepthToImage(const char *fileName, uint32_t height, uint32_t width, uint16_t *data);
    void SaveColorToImage(const char *fileName, uint32_t height, uint32_t width, uint8_t *data);
    void SaveIRToImage(const char *fileName, uint32_t height, uint32_t width, uint8_t *data, uint16_t minVal, uint16_t maxVal);
    void SaveToImage(const char *fileName, uint32_t height, uint32_t width, uint32_t stride, GUID pixelFormat, char *data);

    void SaveCamParameters(const char* path, int camId, CalibFormat format, Calibration& cal, const char* label = "");
    bool LoadCamParameters(const char* path, int camId, CalibFormat format, Calibration& cal, uint16_t numKParams, const char* label = "");
    bool LoadCamParameters(const char* path, const char* camName, CalibFormat format, Calibration& cal, uint16_t numKParams, const char* label = "");

    void UpdateCalibrationWTargetDim(Calibration& cal, uint32_t targetW, uint32_t targetH);   
	void SetDefaultConfig(k4a_device_configuration_t& config);	
	void CreateOutFolders(const std::string& outPath, const std::string& recordingName, const std::string& camName, bool isOfflineCapture = false);
	void CreateOutFolders(const std::string& outPath, const std::string& recordingName, const std::string& camName, int camerasPerFolder, int cameraNum);

    bool SaveCalibrationJsonStringToFile(const char *json_calib_str, size_t json_length, CalibFormat format, const char *output_dir);
}

template <class buff_t>
bool  K4AToFusionUtils::LoadBufferFromBin(const char *fileName, uint32_t height, uint32_t width, buff_t *data)
{
    FILE* file = fopen(fileName, "rb");
    if (file == nullptr)
    {
        std::cout << "!!! Failed to open file " << fileName << " for read" << std::endl;
        return false;
    }

    size_t nPixels = height * width;
    size_t elementsRead = fread(data, sizeof(buff_t), nPixels, file);
    if (elementsRead != nPixels)
    {
        std::cout << "!!! Failed to read all elements. Read " << elementsRead << ", expected " << nPixels << std::endl;
        return false;
    }

    fclose(file);

    return true;
}

template <class buff_t>
bool  K4AToFusionUtils::SaveBufferToBin(const char *fileName, uint32_t height, uint32_t width, buff_t *data)
{
    FILE* file = fopen(fileName, "wb");
    if (file == nullptr)
    {
        std::cout << "!!! Failed to create file " << fileName << " for write" << std::endl;
        return false;
    }

    size_t nPixels = height * width;
    size_t elementsRead = fwrite(data, sizeof(buff_t), nPixels, file);
    if (elementsRead != nPixels)
    {
        std::cout << "!!! Failed to write all elements. Wrote " << elementsRead << ", expected " << nPixels << std::endl;
        return false;
    }

    std::cout << "Success.  Wrote " << elementsRead << " bytes." << std::endl;
    fclose(file);

    return true;
}

void RodriguesToRotation(double rx, double ry, double rz, double R[9]);
void SaveCamParametersFace3D(const char* path, int camId, const char* label, Calibration& cal);
void SaveCamParametersCalibStudio(const char* path, int camId, Calibration& cal);
void SaveCamParametersCalibStudio(const char* path, const char* camName, Calibration& cal);