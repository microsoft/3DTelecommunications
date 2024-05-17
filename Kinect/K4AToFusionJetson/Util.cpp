#include "stdafx.h"
#include "Util.h"
#include "K4ASourceManager.h"

#include <experimental/filesystem>
#include "opencv2/highgui.hpp"
#include <json/json.h>

#define FILENAME_SIZE 1024

#define ValidateKey(keyStr, value) ((value).isMember((keyStr)))

bool ValidateParameterList(Json::Value& json, const char* keys[], size_t nkeys) {

    for (size_t i = 0; i < nkeys; i++) {
        if (!ValidateKey(keys[i], json))
            return false;
    }

    return true;
}

double MillimetersToCentimeters(double v) {
    return v / 10.0f;
}

double MillimetersToFusionMetricSystem(double v) {
    return MillimetersToCentimeters(v);
}

void RodriguesToRotation(double rx, double ry, double rz, double R[9])
{
    double theta = sqrt(rx*rx + ry*ry + rz*rz);
    if (theta == 0)
    {
        R[0] = 1; R[1] = 0; R[2] = 0;
        R[3] = 0; R[4] = 1; R[5] = 0;
        R[6] = 0; R[7] = 0; R[8] = 1;
        return;
    }

    double rxf = (rx / theta);
    double ryf = (ry / theta);
    double rzf = (rz / theta);
    double costh = cos(theta);
    double sinth = sin(theta);

    R[0] = costh + rxf*rxf*(1 - costh);       R[1] = rxf*ryf*(1 - costh) - rzf*sinth;   R[2] = ryf*sinth + rxf*rzf*(1 - costh);
    R[3] = rzf*sinth + rxf*ryf*(1 - costh);   R[4] = costh + ryf*ryf*(1 - costh);       R[5] = -rxf*sinth + ryf*rzf*(1 - costh);
    R[6] = -ryf*sinth + rxf*rzf*(1 - costh);  R[7] = rxf*sinth + ryf*rzf*(1 - costh);   R[8] = costh + rzf*rzf*(1 - costh);

}

bool K4AToFusionUtils::CreateFolder(const char* path)
{
    if(!std::experimental::filesystem::create_directory(path))
    {
        printf("Failed to create directory \"%s\"!\n", path);
        return false;
    }
    return true;
}

void K4AToFusionUtils::IgnoreComments(std::ifstream& ifs)
{
    char val = ifs.peek();
    while (val == '#' || val == '\n')
    {
        std::string comment;
        std::getline(ifs, comment);
        val = ifs.peek();
    }
}


void K4AToFusionUtils::ShowImage(const char* windowName, uint32_t height, uint32_t width, uint16_t cvType, unsigned char* data, int w, int x, int y, double scale)
{
    cv::Mat frame(cv::Size(width, height), cvType, data);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, frame * scale);
    cv::resizeWindow(windowName, w, (int)((double)w/width * height));
    cv::moveWindow(windowName, x, y);
    cv::waitKey(1);
}


void K4AToFusionUtils::ShowImageColorScale(const char* windowName, uint32_t height, uint32_t width, uint16_t cvType, unsigned char* data, int w, int x, int y, double scale)
{
    cv::Mat frameCvt, frameClr;
    cv::Mat frame(cv::Size(width, height), cvType, data);
    frame.convertTo(frameCvt, CV_8UC3);
    cv::applyColorMap(frameCvt, frameClr, cv::COLORMAP_JET);

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, frameClr * scale);
    cv::resizeWindow(windowName, w, (int)((double)w / width * height));
    cv::moveWindow(windowName, x, y);
    cv::waitKey(1);
}


void K4AToFusionUtils::GetDateStr(char* outBuf, size_t bufLen)
{
    time_t curTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm* timeinfo;
    time(&curTime);
    timeinfo = localtime(&curTime);
    strftime(outBuf, bufLen, "%y%m%d_%H%M%S", timeinfo);
}


void K4AToFusionUtils::SaveDepthToImage(const char *fileName, uint32_t height, uint32_t width, uint16_t *data)
{
    cv::Mat img = cv::Mat(height, width, CV_16U, data);
    cv::imwrite(fileName, img);
}


void K4AToFusionUtils::SaveColorToImage(const char *fileName, uint32_t height, uint32_t width, uint8_t *data)
{   
    std::vector<char> pcmImg;

    try
    {
        pcmImg.resize(height * width * 3);
    }
    catch (std::bad_alloc const &)
    {
        printf("Memory allocation failed.\n");
        throw std::exception();
    }    
  
    // convert the 16 bit pixel to 24 bit format
    #pragma omp parallel for
    for (uint32_t i = 0; i < height * width; i++)
    {
        pcmImg[i * 3 + 0] = data[i * 4 + 0];
        pcmImg[i * 3 + 1] = data[i * 4 + 1];
        pcmImg[i * 3 + 2] = data[i * 4 + 2];
    }

    cv::Mat img = cv::Mat(height, width, CV_8UC3, pcmImg.data());
    cv::imwrite(fileName, img);
}


void K4AToFusionUtils::SaveIRToImage(const char *fileName, uint32_t height, uint32_t width, uint8_t *data, uint16_t minVal, uint16_t maxVal)
{
    std::vector<char> pcmImg;

    try
    {
        pcmImg.resize(height * width * 3);
    }
    catch (std::bad_alloc const &)
    {
        printf("Memory allocation failed.\n");
        throw std::exception();
    }

    double range = maxVal - minVal;
    uint16_t* pData = (uint16_t*)data;

    // convert the 16 bit pixel to 24 bit format
    #pragma omp parallel for
    for (uint32_t i = 0; i < height * width; i++)
    {
        char val = (pData[i] > maxVal) ? 255 : (char)((pData[i] - minVal) / range * 255.0);
        pcmImg[i * 3 + 0] = val;
        pcmImg[i * 3 + 1] = val;
        pcmImg[i * 3 + 2] = val;
    }

    cv::Mat img = cv::Mat(height, width, CV_8UC3, pcmImg.data());
    cv::imwrite(fileName, img);    
}

void K4AToFusionUtils::CreateOutFolders(const std::string& outPath, const std::string& recordingName, const std::string& camName, int camerasPerFolder, int cameraNum)
{

	std::vector<std::string> folders;
	std::string recordingFolder = outPath + "/" + recordingName;
	std::string camRootFolder = recordingFolder + "/" + camName;
	folders.push_back(outPath);
	folders.push_back(recordingFolder);

	folders.push_back(camRootFolder);

	folders.push_back(camRootFolder + std::string("/cam") + std::to_string(camerasPerFolder * cameraNum));
	folders.push_back(camRootFolder + std::string("/cam") + std::to_string(camerasPerFolder * cameraNum + 1));

	for (uint32_t i = 0; i < folders.size(); ++i)
		K4AToFusionUtils::CreateFolder(folders[i].c_str());
}

void K4AToFusionUtils::CreateOutFolders(const std::string& outPath, const std::string& recordingName, const std::string& camName, bool isOfflineCapture)
{
	std::vector<std::string> folders;
	std::string recordingFolder = outPath + "/" + recordingName;
	std::string camRootFolder = recordingFolder + std::string("/") + camName;
	folders.push_back(outPath);
	folders.push_back(recordingFolder);

	folders.push_back(camRootFolder);
	folders.push_back(camRootFolder + std::string("/frames"));

	if (!isOfflineCapture)
	{
		folders.push_back(camRootFolder + std::string("_ir"));
		folders.push_back(camRootFolder + std::string("_ir/frames"));

		folders.push_back(camRootFolder + std::string("_ir_uint16"));
		folders.push_back(camRootFolder + std::string("_ir_uint16/frames"));
	}

	folders.push_back(camRootFolder + std::string("_depth"));
	folders.push_back(camRootFolder + std::string("_depth/frames"));

	for (uint32_t i = 0; i < folders.size(); ++i)
		K4AToFusionUtils::CreateFolder(folders[i].c_str());
}

void K4AToFusionUtils::SetDefaultConfig(k4a_device_configuration_t& config)
{
	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // TODO use mjepg or nv12 and convert on GPU
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.synchronized_images_only = true;
}

void SaveCamParametersFace3D(const char* path, int camId, const char* label, Calibration& cal)
{
    char filename[256];
    sprintf(filename, "%s/camExtrin%02d%s.txt", path, camId, label);

    std::ofstream ofs(filename);
    if (!ofs.is_open())
        return;
    
    ofs << std::fixed;    
    ofs.precision(6);

    // width height
    ofs << cal.width << " " << cal.height << std::endl;

    // fx fy cx cy
    ofs << cal.K[0] << " " << cal.K[1] << " " << cal.K[2] << " " << cal.K[3] << std::endl;

    // distortion paramters
    for (int i = 0; i < 8; ++i)
        ofs << cal.dist[i] << std::endl;

    for (int i = 8; i < 17; ++i)
        ofs << 0.0 << std::endl;

    ofs << std::endl;

    // rotation
    ofs << cal.R[0] << " " << cal.R[1] << " " << cal.R[2] << std::endl;
    ofs << cal.R[3] << " " << cal.R[4] << " " << cal.R[5] << std::endl;
    ofs << cal.R[6] << " " << cal.R[7] << " " << cal.R[8] << std::endl;

    ofs << std::endl;

    // translation
    ofs << cal.t[0] << " " << cal.t[1] << " " << cal.t[2] << std::endl;

    // zscale zshift
    ofs << cal.zScale << " " << cal.zShift << std::endl;

    ofs.close();
}


void SaveCamParametersCalibStudio(const char* path, int camId, Calibration& cal)
{
    char camName[15];
    sprintf(camName, "cam%d.txt", camId);
    SaveCamParametersCalibStudio(path, camName, cal);
}

void SaveCamParametersCalibStudio(const char* path, const char* camName, Calibration& cal)
{
    char filename[256];
    sprintf(filename, "%s/%s.txt", path, camName);
    *LOGGER() << Logger::Info << "Saving calibration to " << filename << Logger::endl;
    std::ofstream ofs(filename, std::ios::out);
    if (!ofs.is_open())
    {
        *LOGGER() << Logger::Error << "ERROR.  Could not open stream for writing!" << Logger::endl;
        return;
    }

    ofs << std::fixed;
    ofs.precision(8);

    // One camera in this config file, second is dummy variable
    ofs << "1 0" << std::endl;

    // CameraName width height
    ofs << camName << " " << cal.width << " " << cal.height << std::endl;

    // fx fy 0 cx cy <distortion params>
    ofs << cal.K[0] << " " << cal.K[1] << " 0 " << cal.K[2] << " " << cal.K[3];
    for (int i = 0; i < 5; ++i)
        ofs << " " << cal.dist[i];
    ofs << std::endl;

    // rotation (rodrigues) translation
    ofs << "0 0 0 " << cal.t[0] << " " << cal.t[1] << " " << cal.t[2] << std::endl;

    // color scale & bias
    ofs << cal.colorScale[0] << " " << cal.colorScale[1] << " " << cal.colorScale[2] << " " << cal.colorBias[0] << " " << cal.colorBias[1] << " " << cal.colorBias[2];

    ofs.close();
}


void K4AToFusionUtils::SaveCamParameters(const char* path, int camId, CalibFormat format, Calibration& cal, const char* label)
{
    switch (format)
    {
    case CalibFormat_Face3D:
        SaveCamParametersFace3D(path, camId, label, cal);
        break;
    case CalibFormat_CalibStudio:
        SaveCamParametersCalibStudio(path, camId, cal);
        break;
    case CalibFormat_3DTM:
        // TODO: Implement saving 3DTM Format. Bug #9683 in ADO
        printf("ERROR: Saving 3DTM Format not yet implemented");
        break;
    default:
        printf("WARNING: Unknown format %d\n", (int)format);
        break;
    }
}


bool LoadCamParametersFace3D(const char* path, const char* camName, Calibration& cal, uint16_t numKParams, const char* label)
{
    char filename[256];
    sprintf(filename, "%s/camExtrin%s%s.txt", path, camName, label);

    std::ifstream calibIFS(filename);
    if (!calibIFS.is_open())
        return false;

    memset(&cal, 0, sizeof(Calibration));

    // width height
    calibIFS >> cal.width >> cal.height;

    // fx fy cx cy
    calibIFS >> cal.K[0] >> cal.K[1] >> cal.K[2] >> cal.K[3];  

    // distortion paramters
    int numParams = numKParams + 2; // 3kt == 3 radial + 2 tangential, 6kt == 6 radial + 2 tangential 
    for (int i = 0; i < numParams; ++i)
        calibIFS >> cal.dist[i];

    double otherParams;
    for (int i = numParams; i < 17; ++i)
        calibIFS >> otherParams;

    // rotation
    calibIFS >> cal.R[0] >> cal.R[1] >> cal.R[2];
    calibIFS >> cal.R[3] >> cal.R[4] >> cal.R[5];
    calibIFS >> cal.R[6] >> cal.R[7] >> cal.R[8];


    // translation
    calibIFS >> cal.t[0] >> cal.t[1] >> cal.t[2];

    // zscale zshift
    calibIFS >> cal.zScale >> cal.zShift;    

    calibIFS.close();

    return true;
}


bool LoadCamParametersCalibStudio(const char* path, const char* camName, Calibration& cal, uint16_t numKParams)
{   
    char filename[256];
    sprintf(filename, "%s/%s.txt", path, camName);

    *LOGGER() << Logger::Info << "Loading CalibStudio camera parameters for " << camName << " from " << filename << Logger::endl;
    std::ifstream calibIFS(filename);
    if (!calibIFS.is_open())
    {
        *LOGGER() << Logger::Error << "ERROR.  Could not open " << filename << Logger::endl;
        return false;
    }

    memset(&cal, 0, sizeof(Calibration));
    cal.zScale = 1.0;
    cal.zShift = 0.0;

    //// width height 
    bool found = false;
    int numCams = 0, dummy = 0;
    calibIFS >> numCams >> dummy;
    while (!calibIFS.eof() && !found)
    {
        std::string cam;
        calibIFS >> cam >> cal.width >> cal.height;
        found = cam == camName;

        // fx fy 0 cx cy <distortion params>
        int empty;
        calibIFS >> cal.K[0] >> cal.K[1] >> empty >> cal.K[2] >> cal.K[3];

        int numParams = numKParams + 2; // 3kt == 3 radial + 2 tangential, 6kt == 6 radial + 2 tangential 
        for (int i = 0; i < numParams; ++i)
            calibIFS >> cal.dist[i];

        // rotation (rodrigues)
        double rodrigues[3];
        calibIFS >> rodrigues[0] >> rodrigues[1] >> rodrigues[2];
        RodriguesToRotation(rodrigues[0], rodrigues[1], rodrigues[2], cal.R);

        // translation
        calibIFS >> cal.t[0] >> cal.t[1] >> cal.t[2];

        //// color bias
        //"1 1 1 0 0 0"        
        calibIFS >> dummy >> dummy >> dummy >> dummy >> dummy >> dummy;
    }

    calibIFS.close();

    if (!found)
    {
        printf("Specifid camera [%s] not found in calib file\n", camName);
    }

    return found;
}

bool Validate3DTMFormatForCamera(Json::Value &camera) {
    const char *intrinsics_req[] = {"resolution", "codx", "cody", "fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2"};
    const char *extrinsics_req[] = {"rotation", "translation"};

    bool valid = true;

    valid = valid && ValidateParameterList(camera["intrinsics"], intrinsics_req, 12);
    valid = valid && ValidateParameterList(camera["extrinsics"], extrinsics_req, 2);

    if(valid) {
        valid = valid && camera["extrinsics"]["rotation"].size() == 3 && camera["extrinsics"]["rotation"][0].size() == 3 && camera["extrinsics"]["rotation"][1].size() == 3 && camera["extrinsics"]["rotation"][2].size() == 3;
        valid = valid && camera["extrinsics"]["translation"].size() == 3;
    }

    return valid;
}

bool LoadCamParameters3DTMFormat(const char* calib_dir, const char *camName, const char* camera_type, Calibration& cal, uint16_t numKParams)
{
    std::string calib_filename = std::string(calib_dir) + "/calibCameras3DTM.json";
    std::ifstream calibIFS(calib_filename);
    if (!calibIFS.is_open())
    {
        LOGGER()->error("LoadCamParameters3DTMFormat", "Can't open 3DTM Json calibration file %s", calib_filename.c_str());
        return false;
    }

    *LOGGER() << Logger::Info << "Loading 3DTM format camera parameters for camera with serial " << camName << " and type \"" << camera_type << "\" from " << calib_filename << Logger::endl;

    unsigned int camera_idx;
    bool found = false;
    
    // Setting calibration to 0
    memset(&cal, 0, sizeof(Calibration));

    // Parsing Json file
    Json::Value root;

    try {
        calibIFS >> root;
    } catch(const std::exception &e) {
        LOGGER()->error("LoadCamParameters3DTMFormat", "Invalid 3DTM Format in %s", calib_filename.c_str());
        return false;
    }

    calibIFS.close();

    // Searching for camera with serial matching camName
    for(unsigned int i = 0; i < root["inputCameras"].size(); i++) {
        if(ValidateKey("device", root["inputCameras"][i]) && ValidateKey("serial", root["inputCameras"][i]["device"])) {
            std::string serial = root["inputCameras"][i]["device"]["serial"].asString();

            if(serial == camName) {
                found = true;
                camera_idx = i;
                break;
            }
        } else {
            LOGGER()->error("LoadCamParameters3DTMFormat", "Invalid 3DTM Format!Could not find device serial in %s to detect camera %s", calib_filename.c_str(), camName);
            return false;
        }
    }

    if(!found) {
        LOGGER()->error("LoadCamParameters3DTMFormat", "Could not find calibration for camera with serial %s in 3DTM Json calibration file %s", camName, calib_filename.c_str());
        return false;
    }

    if(!ValidateKey(camera_type, root["inputCameras"][camera_idx])) {
        LOGGER()->error("LoadCamParameters3DTMFormat", "Invalid 3DTM Format for camera %s in file %s.Unable to detect camera type %s", camName, calib_filename.c_str(), camera_type);
        return false;
    }
    
    // Creating reference to the appropriate camera
    Json::Value& camera = root["inputCameras"][camera_idx][camera_type];

    if(!Validate3DTMFormatForCamera(camera)) {
        LOGGER()->error("LoadCamParameters3DTMFormat, ", "Invalid 3DTM Format for camera %s in file %s", camName, calib_filename.c_str());
        return false;
    }

    // Depth cameras are likely going to have a scale and offset parameter, depending on the calibration algorithm.
    // We verify if the Json file contains those values for the camera and copy them. Otherwise, the values default to 1.0 and 0.0.
    cal.zScale = ValidateKey("scale", camera) ? camera["scale"].asDouble() : 1.0;
    cal.zShift = ValidateKey("offset", camera) ? camera["offset"].asDouble() : 0.0;
    
    cal.width = camera["intrinsics"]["resolution"]["width"].asDouble();
    cal.height = camera["intrinsics"]["resolution"]["height"].asDouble();

    // Copying focal lengths and principal point into K	
    cal.K[0] = camera["intrinsics"]["fx"].asDouble();
    cal.K[1] = camera["intrinsics"]["fy"].asDouble();
    cal.K[2] = camera["intrinsics"]["cx"].asDouble();
    cal.K[3] = camera["intrinsics"]["cy"].asDouble();

    // Copying camera rotation matrix to array
    double rTotalCheck = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // Copying rotation to linearized matrix in array coordinates
            double temp = camera["extrinsics"]["rotation"][i][j].asDouble();
            cal.R[j+i*3] = temp;            
            rTotalCheck += temp;
        }
    }

    //extra calib file validity check on rotation. All zero rotation matrix is non-valid
    if (rTotalCheck == 0)
    {
        LOGGER()->error("LoadCamParameters3DTMFormat","camera rotation matrix can't be all 0's");
        return false;
    }

    // Copying translation and converting millimeters to centimeters
    for (int i = 0; i < 3; i++)
    {
        cal.t[i] = MillimetersToFusionMetricSystem(camera["extrinsics"]["translation"][i].asDouble());
    }

    // Copying radial distortion coefficients and tangential parameters 
    const char* k_names[] = { "k1", "k2", "p1", "p2", "k3"};

    for (int i = 0; i < 5; i++)
    {
        cal.dist[i] = camera["intrinsics"][k_names[i]].asDouble();
    }

    // If we have 6 radial coefficients instead of 3, cal.dist should contain a total of 8 parameters (6 radial + 2 tangential) instead of 5 (3 radial + 2 tangential).
    // We thus verify if the camera Json has those intrinsics and add them accordingly. 
    // IMPORTANT: even though we'll be reading the values into cal.dist, the input parameter numKParams
    // may override the number of radial distortion coefficients due to backwards compatibility
    // TODO: Thiago, determine if we can simply assign cal.numKParams as 3 or 6 here and exclude numKParams 
    // as a calling parameter. Bug #9684 in ADO
    const char* k_names_extra[] = { "k4", "k5", "k6" };
    
    if (ValidateParameterList(camera["intrinsics"], k_names_extra, 3)) {
        for (int i = 5, k = 0; i < 8; i++, k++)
        {
            cal.dist[i] = camera["intrinsics"][k_names_extra[k]].asDouble();
        }
    }

    cal.numKParams = numKParams;

    for (int i = 0; i < 3; i++) {
        cal.colorScale[i] = 1.0;
        cal.colorBias[i] = 0.0;
    }

    return true;
}

bool K4AToFusionUtils::LoadCamParameters(const char* path, int camId, CalibFormat format, Calibration& cal, uint16_t numKParams, const char* label)
{
    char camName[15];
    sprintf(camName, "cam%d", camId);
    return LoadCamParameters(path, camName, format, cal, numKParams, label);
}

bool K4AToFusionUtils::LoadCamParameters(const char* path, const char* camName, CalibFormat format, Calibration& cal, uint16_t numKParams, const char* label)
{
    bool success = false;

    *LOGGER() << Logger::Info << "Loading camera parameters. Format: " << (int)format << Logger::endl;

    switch (format)
    {
    case CalibFormat_Face3D:
        *LOGGER() << Logger::Info << "Loading Face3D File format" << Logger::endl;
        success = LoadCamParametersFace3D(path, camName, cal, numKParams, label);
        break;
    case CalibFormat_CalibStudio:
        *LOGGER() << Logger::Info << "Loading CalibStudio File format" << Logger::endl;
        success = LoadCamParametersCalibStudio(path, camName, cal, numKParams);
        break;
    case CalibFormat_3DTM:
        *LOGGER() << Logger::Info << "Loading 3DTM File format" << Logger::endl;
        success = LoadCamParameters3DTMFormat(path, camName, label, cal, numKParams);
        break;
    default:
        LOGGER()->warning("K4AToFusionUtils::LoadCamParameters", "Unknown format %d", (int)format);
        break;
    }

    return success;
}


bool ParseJsonFromString(const char *rawJson, size_t rawJsonLength, Json::Value &root) {
    JSONCPP_STRING err;
 
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(rawJson, rawJson + rawJsonLength, &root, &err)) {
        *LOGGER() << Logger::Error << "Error parsing Json" << Logger::endl;
        return false;
    }

    return true;
}

bool Save3DTMCalibrationJsonFormat(const std::string & calib_dir, const Json::Value & json_converted_calib)
{
    bool success = false;
	std::string json_converted_calib_filename = calib_dir + "\\calibCameras3DTM.json";

	LOGGER()->info("Writing converted Json format to file %s", json_converted_calib_filename.c_str());

	Json::StreamWriterBuilder wbuilder;
	std::string document = Json::writeString(wbuilder, json_converted_calib);
	std::ofstream ofs(json_converted_calib_filename, std::ios::trunc);

    if(ofs.is_open()){
        ofs << std::fixed;
        ofs.precision(6);
        ofs << document << std::endl;

        success = true;
    }

    return success;
}

bool K4AToFusionUtils::SaveCalibrationJsonStringToFile(const char *json_calib_str, size_t json_length, CalibFormat format, const char *output_dir) {
    Json::Value json;
    bool success = false;

    switch(format) {
        case CalibFormat_3DTM:
            *LOGGER() << Logger::Info << "Parsing 3DTM from raw string before writing to file as a sanity check measure..." << Logger::endl;
            if(!ParseJsonFromString(json_calib_str, json_length, json)) {
                *LOGGER() << Logger::Error << "Unable to parse 3DTM format Json from string!" << Logger::endl;
                break;
            }
            success = Save3DTMCalibrationJsonFormat(output_dir, json);       
            break;
        default:
            *LOGGER() << Logger::Error << "Calibration format " << format << " not compatible with Json" << Logger::endl;
            break;
    }

    return success;
}


void K4AToFusionUtils::UpdateCalibrationWTargetDim(Calibration& cal, uint32_t targetW, uint32_t targetH)
{
    cal.K[2] = (targetW - cal.width) / 2.0 + cal.K[2];
    cal.K[3] = (targetH - cal.height) / 2.0 + cal.K[3];

    cal.width = targetW;
    cal.height = targetH;
}

