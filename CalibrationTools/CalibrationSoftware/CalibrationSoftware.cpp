// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include <iostream>
#include <string>
#include <filesystem>
#include <stdlib.h>
#include <ctime>
#include <json/json.h>
#include "3dtm_version.h"
#include "Config.h" 
#include "CalibrationSoftwareControlPanelConnector.h"
#include "PAICalibration.h"
#include "ConvertPAICalibration.h"

struct CalibrationProgressState {
	CalibrationSoftwareControlPanelConnector* CSCPC;

	float DepthError;
	float ReprojectionError;
	float MultiViewMisalignment;
};

CConfig* LoadConfigFile(const char* argv[], int argc, std::string& config_filename)
{
	CConfig* conf = NULL;

	for (int i = 1; i < argc; ++i)
	{
		if (_stricmp(argv[i], "--config") == 0 && i + 1 < argc) config_filename = argv[i + 1];
	}

	// if a config wasn't specified on the command line
	if (config_filename.empty())
	{
		std::cout << "No config file specified on command line.  Using 3DTelecommunications_dir environment variable." << std::endl;
		size_t env_var_3dtm_len;
		char* env_var_3dtm = NULL;

		errno_t err = _dupenv_s(&env_var_3dtm, &env_var_3dtm_len, "3DTelecommunications_dir");

		// Try the System directory
		if (env_var_3dtm != NULL)
		{
			std::string tmdir = std::string(env_var_3dtm);

			tmdir += "/3DTelecommunications.cfg";
			if (std::filesystem::exists(std::filesystem::path(tmdir)))
			{
				std::cout << "Using " << tmdir << std::endl;
				config_filename = tmdir;
			}
			// if a config doesn't exist there, look for the basic .default version to create one
			else if (std::filesystem::exists(std::filesystem::path(tmdir + ".default")))
			{
				std::cerr << "!!! No config file found.  Copying the default in place for use." << std::endl;
				std::filesystem::copy_file(std::filesystem::path(tmdir + ".default"), std::filesystem::path(tmdir));
				config_filename = tmdir;
			}
		}
	}

	if (!config_filename.empty())
	{
		conf = new CConfig();
		std::cout << "!!! Using configuration file : " << config_filename << std::endl;
		auto loaded = conf->Load(config_filename);
		if (!loaded)
		{
			std::cerr << "Attempted to load config file " << config_filename << ", but failed" << std::endl;
			delete conf;
			return NULL;
		}
	}

	return conf;
}

bool ComputeLogFilePaths(CConfig* conf, std::string& folderPath, std::string& logFilePath, std::string& errorFilePath)
{
	size_t documentsPath_len;
	char* documentsPath = NULL;
	errno_t err = _dupenv_s(&documentsPath, &documentsPath_len, "APPDATA");

	if (err) {
		documentsPath = NULL;
		documentsPath_len = 0;
	}

	if (documentsPath != NULL) {
		std::string basename = "CalibrationSoftware";
		std::string section = "Calibration";

		folderPath = std::string(documentsPath) + conf->GetValueWithDefault(section.c_str(), "LogRelativePath", "\\..\\LocalLow\\Microsoft Research\\CalibrationLogs");
		logFilePath = folderPath + "\\" + conf->GetValueWithDefault(section.c_str(), "LogFilename", (basename + ".log").c_str());
		errorFilePath = folderPath + "\\" + conf->GetValueWithDefault(section.c_str(), "ErrorFilename", (basename + ".error").c_str());

		return true;
	}

	return false;
}

void SetupLogFiles(CConfig* conf)
{
	std::string folderPath, logFilePath, errorFilePath;

	if (ComputeLogFilePaths(conf, folderPath, logFilePath, errorFilePath))
	{
		//create dir if not exist
		std::error_code ec;
		std::filesystem::create_directories(folderPath, ec);
		if (!ec)
		{
			LOGGER()->set_verbosity((Logger::Verbosity)conf->GetValueWithDefault<int>("Calibration", "Verbosity", 3));
			LOGGER()->set_filename(logFilePath);
			LOGGER()->set_error_filename(errorFilePath);
			LOGGER()->info("CalibrationSoftware stdout/stderr redirected to %s and %s\n", logFilePath.c_str(), errorFilePath.c_str());

			LOGGER()->info("CalibrationSoftware Output Log start");
			LOGGER()->error("CalibrationSoftware::SetupLogFiles", "Error Log start");
		}
		else
		{
			LOGGER()->error("CalibrationSoftware", "Log FOLDER creation Failed for path% s :% s", folderPath.c_str(), ec.message().c_str());
		}
	}
}

const std::string DummyJson() {
	const std::string rawJson = R"({"Intrinsics": 20, "Extrinsics": 3.0})";
	const auto rawJsonLength = static_cast<int>(rawJson.length());
	constexpr bool shouldUseOldWay = false;
	JSONCPP_STRING err;
	Json::Value root;

	if (shouldUseOldWay) {
		Json::Reader reader;
		reader.parse(rawJson, root);
	}
	else {
		Json::CharReaderBuilder builder;
		const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
		if (!reader->parse(rawJson.c_str(), rawJson.c_str() + rawJsonLength, &root,
			&err)) {
			std::cout << "error" << std::endl;
			return "";
		}
	}

	Json::StreamWriterBuilder wbuilder;
	std::string converted_string = Json::writeString(wbuilder, root);

	return converted_string;
}

void ProgressCallback(void* state, uint32_t frameIndex, uint32_t endFrame) {
	std::string progress_msg;
	std::stringstream ss;

	if (endFrame != 0 && endFrame != std::numeric_limits<uint32_t>::max()) {
		ss << "Processed frame " << frameIndex << "/" << endFrame << ", please wait...";
	}
	else {
		ss << "Processed frame " << frameIndex << ", please wait...";
	}

	progress_msg = ss.str();

	if (state != NULL) {
		CalibrationProgressState* cp_state = reinterpret_cast<CalibrationProgressState*>(state);

		cp_state->CSCPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_SOFTWARE_PROCESSING, progress_msg.c_str(), progress_msg.size());
	}
}

void CalibrationCompletedCallback(void* state, float ReprojectionError, float DepthError, float MultiViewMisalignment) {
	if (state != NULL) {
		CalibrationProgressState* cp_state = reinterpret_cast<CalibrationProgressState*>(state);

		cp_state->DepthError = DepthError;
		cp_state->ReprojectionError = ReprojectionError;
		cp_state->MultiViewMisalignment = MultiViewMisalignment;
	}

	LOGGER()->info("Calibration completed callback: Mean ReprojectionError %f pixels. Mean Depth Error %f mm. MultiViewMisalignment %f mm",
		ReprojectionError, DepthError, MultiViewMisalignment);
}

int InvokePAICalibration(CConfig* conf, CalibrationSoftwareControlPanelConnector* CSCPC) {

	Json::Value params;
	std::string extra_params;
	std::string align_method;
	std::string command = "RunCalibrationApi";
	std::list<std::string> command_args_list;
	CalibrationProgressState* cp_state = new CalibrationProgressState();

	cp_state->CSCPC = CSCPC;

	// Required parameters
	params["rigType"] = conf->GetValueWithDefault("Calibration", "RigType", "Kinect");
	params["maximumTimeSeparationOfSubframes"] = conf->GetValueWithDefault("Calibration", "MaximumTimeSeparationOfSubframes", 2000);
	params["calibration-mode"] = conf->GetValueWithDefault("Calibration", "CalibrationMode", "stage2"); // Default is to compute extrinsics only. Using stage2 gives us that
	params["mkvsDir"] = conf->GetValueWithDefault("Calibration", "CalibrationWorkingDirectory", "C:\\3DTelecommunications\\DATA\\Calibration");
	params["outDir"] = conf->GetValueWithDefault("Calibration", "CalibrationWorkingDirectory", "C:\\3DTelecommunications\\DATA\\Calibration");
	params["objectSize"] = conf->GetValueWithDefault(conf->GetValueWithDefault("Calibration", "CalibrationBoard", "CalibrationBoard-A0"), "ObjectSize", 0.14f);
	params["clipMetricsFilePath"] = std::string(conf->GetValueWithDefault("Calibration", "CalibrationWorkingDirectory", "C:\\3DTelecommunications\\DATA\\Calibration")) + "\\calibrationMetrics.csv";
	params["numOfDevices"] = conf->GetValueWithDefault("DepthGeneration", "DepthCameraCount", 10);
	
	if (conf->HasKey("Calibration", "DevicesCalibrationJson")) {
		params["devicesCalibrationJson"] = conf->GetValue<std::string>("Calibration", "DevicesCalibrationJson");
		LOGGER()->warning("InvokePAICalibration", "Using devices calibration Json: %s. Intrinsics will be loaded from it and take precedence over computed and/or factory values", 
			params["devicesCalibrationJson"].asString().c_str());
	}
	// Copying extra parameters, if any
	extra_params = conf->GetValueWithDefault("Calibration", "ExtraParams", "");

	// Adding the command name as the first one in the parameter list
	command_args_list.push_back(command);

	// Copying required valued paramaters into the command argument array
	for (auto p = params.begin(); p != params.end(); p++) {
		std::string param = "--" + p.key().asString();
		std::string param_value = p->asString();

		command_args_list.push_back(param);
		command_args_list.push_back(param_value);
	}

	// Splitting extra parameters into the command argument list
	if (extra_params != "") {
		std::stringstream ss(extra_params);
		std::string p;
		while (ss >> p) {
			command_args_list.push_back(p);
		}
	}

	// Getting world alignment method (--useImuToAlignYAxisToGravity by default). Unlike the other parameters, doesn't have an associated value
	if (conf->HasKey("Calibration", "YAlignmentMethod")) {
		align_method = conf->GetValueWithDefault("Calibration", "YAlignmentMethod", "useImuToAlignYAxisToGravity");
		LOGGER()->warning("InvokePAICalibration", "Using option --useImuToAlignYAxisToGravity");
		command_args_list.push_back("--" + align_method);
	}

	// Allocating main-style command argument array based on the command argument list
	int ncommand_args = command_args_list.size();
	char** command_args = (char**)calloc(ncommand_args, sizeof(char*));

	// Copying command argument list into the command argument array
	LOGGER()->info("Preparing PAI command arguments");
	int i = 0;
	for (auto p = command_args_list.begin(); p != command_args_list.end(); i++, p++) {
		command_args[i] = (char*)calloc(p->size() + 1, sizeof(char));

		strcpy_s(command_args[i], p->size() + 1, p->c_str());

		LOGGER()->info("Arg %02d: %s", i, command_args[i]);
	}

	// Redirecting outputs to log files
	std::string folderPath, logFilePath, errorFilePath;

	if (ComputeLogFilePaths(conf, folderPath, logFilePath, errorFilePath)) {
		LOGGER()->warning("CalibrationSoftware", "Redirecting STDOUT and STDERR of calibration command %s to %s and %s", command.c_str(),
			logFilePath.c_str(), errorFilePath.c_str());
	}
	else
	{
		LOGGER()->error("CalibrationSoftware", "Unable to redirect STDOUT and STDERR of calibration command %s", command.c_str());
	}

	// Running calibration API passing the desired parameters as main-style command arguments. Providing callbacks to track
	// calibration progress and completion
	int ret = RunCalibrationApi(ncommand_args, (const char**)command_args, cp_state, ProgressCallback, CalibrationCompletedCallback);

	if (ret == 0) {
		Json::Value metrics;
		metrics["DepthError"] = cp_state->DepthError;
		metrics["ReprojectionError"] = cp_state->ReprojectionError;
		metrics["MultiViewMisalignment"] = cp_state->MultiViewMisalignment;
		// Appending extra parameters to parameter Json to store them in the 3DTM calibration format Json
		if (extra_params != "") {
			params["ExtraParams"] = extra_params;
		}
		ret = ConvertPAICalibration(*conf, params, metrics);
	}

	// Releasing command argument memorymemory
	for (auto i = 0; i < ncommand_args; i++) {
		free(command_args[i]);
	}

	free(command_args);
	delete cp_state;

	return ret;
}


std::string slurp(const char* filename)
{
	std::ifstream in;
	std::stringstream sstr;

	in.open(filename, std::ifstream::in | std::ifstream::binary);
	if (in.is_open()) {
		sstr << in.rdbuf();
		in.close();
	}
	return sstr.str();
}

std::string FetchCalibrationResultAsJsonString(CConfig* conf) {
	std::string json_string;

	std::string calib_filename = std::string(conf->GetValueWithDefault("Calibration", "CalibrationWorkingDirectory", "C:\\3DTelecommunications\\DATA\\Calibration")) + "\\calibCameras3DTM.json";
	LOGGER()->info("Fetching calibration file %s", calib_filename.c_str());
	json_string = slurp(calib_filename.c_str());

	return json_string;
}

void SendCalibrationResultToControlPanel(std::string& json_string, CalibrationSoftwareControlPanelConnector* CSCPC)
{
	const int payload_offset = sizeof(int);
	const int message_len = (int)json_string.length() + payload_offset;
	char* message = new char[message_len];

	((int*)message)[0] = (int)json_string.length();

	LOGGER()->info("SendStatusUpdate(CPC_STATUS::CALIBRATION_SOFTWARE_RESULT)");

	// Copying Json string as the payload to the message
	memcpy_s(message + payload_offset, (size_t)message_len - payload_offset, json_string.c_str(), json_string.length());

	LOGGER()->info("Message length: %d Json length %d", message_len, json_string.length());

	CSCPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_SOFTWARE_RESULT, message, message_len);

	delete[] message;
}

int main(int argc, const char* argv[])
{
	CalibrationSoftwareControlPanelConnector* CSCPC = NULL;
	CConfig* conf = NULL;
	std::string controlPanelIP = "127.0.0.1";
	std::string myInterfaceIP = "127.0.0.1";
	std::string eventPort = "14520";
	std::string statusPort = "14512";
	Json::Value test_json = DummyJson();
	std::string config_filename;
	bool standalone = false;


	for (int i = 1; i < argc && !standalone; ++i)
	{
		if (_stricmp(argv[i], "--standalone") == 0)
			standalone = true;
	}

	conf = LoadConfigFile(argv, argc, config_filename);

	if (conf == NULL) {
		std::cerr << "Unable to find configuration file!! Exiting..." << std::endl;
		return -1;
	}

	SetupLogFiles(conf);

	controlPanelIP = conf->GetValueWithDefault<std::string>("Network", "ControlPanelIPAddress", controlPanelIP);
	myInterfaceIP = conf->GetValueWithDefault<std::string>("Network", "CalibrationSoftwareIPAddress", myInterfaceIP);
	eventPort = conf->GetValueWithDefault<std::string>("Ports", "EventPort", eventPort);
	statusPort = conf->GetValueWithDefault<std::string>("Ports", "StatusPort", statusPort);

	if (!standalone) {
		CSCPC = new CalibrationSoftwareControlPanelConnector(controlPanelIP, eventPort, myInterfaceIP, statusPort);
		CSCPC->SetVersionInformation(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_COMMITS, std::string(VERSION_BRANCH_NAME), std::string(VERSION_DESCRIPTION), std::string(VERSION_SHA1));
	}

	int ret = InvokePAICalibration(conf, CSCPC);

	if (!standalone) {
		std::string json_string = ""; // Empty string denotes calibration error

		if (ret == 0) {
			json_string = FetchCalibrationResultAsJsonString(conf);
		}

		SendCalibrationResultToControlPanel(json_string, CSCPC);

		delete CSCPC;
	}

	LOGGER()->close();

	return 0;
}
