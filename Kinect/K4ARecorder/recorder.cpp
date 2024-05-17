// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "recorder.h"
#include <ctime>
#include <chrono>
#include <atomic>
#include <iostream>
#include <algorithm>
#include "Logger.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>

using namespace std::chrono;

inline static uint32_t k4a_convert_fps_to_uint(k4a_fps_t fps)
{
    uint32_t fps_int;
    switch (fps)
    {
    case K4A_FRAMES_PER_SECOND_5:
        fps_int = 5;
        break;
    case K4A_FRAMES_PER_SECOND_15:
        fps_int = 15;
        break;
    case K4A_FRAMES_PER_SECOND_30:
        fps_int = 30;
        break;
    default:
        fps_int = 0;
        break;
    }
    return fps_int;
}

// call k4a_device_close on every failed CHECK
#define CHECK(x, device)                                                                 \
    {                                                                                    \
        auto retval = (x);                                                               \
        if (retval)                                                                      \
        {                                                                                \
            LOGGER()->error("K4ARecorder", "Runtime error: %d returned %d", x, retval);  \
            k4a_device_close(device);                                                    \
            return 1;                                                                    \
        }                                                                                \
    }

std::atomic_bool exiting(false);

void set_color_param(k4a_device_t &device,
                     k4a_color_control_command_t command,
                     const char *command_name,
                     int32_t value,
                     int32_t defaultValue,
                     bool defaultAuto)
{
    k4a_color_control_mode_t read_mode;
    int32_t read_value;

    if (value != defaultValue)
    {
        if (K4A_FAILED(k4a_device_set_color_control(device, command,
                                                    K4A_COLOR_CONTROL_MODE_MANUAL, value)))
        {
            LOGGER()->error("K4ARecorder", "Runtime error: k4a_device_set_color_control() failed for manual %s", command_name);
        }
    }
    else if (defaultAuto)
    {
        if (K4A_FAILED(k4a_device_set_color_control(device, command,
                                                    K4A_COLOR_CONTROL_MODE_AUTO,
                                                    0)))
        {
            LOGGER()->error("K4ARecorder", "Runtime error: k4a_device_set_color_control() failed for auto %s", command_name);
        }
    }

    k4a_device_get_color_control(device, command, &read_mode, &read_value);
    LOGGER()->info("Current: %s value %d", (read_mode == K4A_COLOR_CONTROL_MODE_AUTO ? "AUTO" : "MANUAL"), read_value);
}

int do_recording(uint8_t device_index,
                 char *recording_filename,
                 int recording_length,
                 k4a_device_configuration_t *device_config,
                 bool record_imu,
                 int32_t absoluteExposureValue,
                 int32_t whiteBalance,
                 int32_t brightness,
                 int32_t contrast,
                 int32_t saturation,
                 int32_t sharpness,
                 int32_t gain,
                 void *state,
                 bool send_frame_counter,
                 void (*onFrameUpdate)(void *state, bool send_frame_counter))
{
    seconds recording_length_seconds(recording_length);
    const uint32_t installed_devices = k4a_device_get_installed_count();
    if (device_index >= installed_devices)
    {
        LOGGER()->error("K4ARecorder", "Device not found.");
        return 1;
    }

    k4a_device_t device;
    if (K4A_FAILED(k4a_device_open(device_index, &device)))
    {
        LOGGER()->error("K4ARecorder", "Runtime error: k4a_device_open() failed.");
    }

    char serial_number_buffer[256];
    size_t serial_number_buffer_size = sizeof(serial_number_buffer);
    CHECK(k4a_device_get_serialnum(device, serial_number_buffer, &serial_number_buffer_size), device);

    LOGGER()->info("Device serial number: %s", serial_number_buffer);

    k4a_hardware_version_t version_info;
    CHECK(k4a_device_get_version(device, &version_info), device);

    LOGGER()->info("Device version: %s", (version_info.firmware_build == K4A_FIRMWARE_BUILD_RELEASE ? "Rel" : "Dbg"));
    LOGGER()->info("C: %d.%d.%d", version_info.rgb.major, version_info.rgb.minor, version_info.rgb.iteration);
    LOGGER()->info("D: %d.%d.%d[%d.%d]", version_info.depth.major, version_info.depth.minor, version_info.depth.iteration, 
                                         version_info.depth_sensor.major, version_info.depth_sensor.minor);
    LOGGER()->info("A: %d.%d.%d", version_info.audio.major, version_info.audio.minor, version_info.audio.iteration);


    uint32_t camera_fps = k4a_convert_fps_to_uint(device_config->camera_fps);

    if (camera_fps <= 0 || (device_config->color_resolution == K4A_COLOR_RESOLUTION_OFF &&
                            device_config->depth_mode == K4A_DEPTH_MODE_OFF))
    {
        LOGGER()->error("K4ARecorder", "Either the color or depth modes must be enabled to record.");
        return 1;
    }

    set_color_param(device,
                    K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
                    "exposure",
                    absoluteExposureValue,
                    defaultExposureAuto,
                    true);
    set_color_param(device, K4A_COLOR_CONTROL_WHITEBALANCE, "white balance", whiteBalance, defaultWhiteBalance, true);
    set_color_param(device, K4A_COLOR_CONTROL_BRIGHTNESS, "brightness", brightness, defaultBrightness, false);
    set_color_param(device, K4A_COLOR_CONTROL_CONTRAST, "contrast", contrast, defaultContrast, false);
    set_color_param(device, K4A_COLOR_CONTROL_SATURATION, "saturation", saturation, defaultSaturation, false);
    set_color_param(device, K4A_COLOR_CONTROL_SHARPNESS, "sharpness", sharpness, defaultSharpness, false);
    set_color_param(device, K4A_COLOR_CONTROL_GAIN, "gain", gain, defaultGainAuto, false);

    CHECK(k4a_device_start_cameras(device, device_config), device);
    if (record_imu)
    {
        CHECK(k4a_device_start_imu(device), device);
    }

    LOGGER()->info("Device started");

    k4a_record_t recording;
    if (K4A_FAILED(k4a_record_create(recording_filename, device, *device_config, &recording)))
    {
        LOGGER()->error("K4ARecorder", "Unable to create recording file: %s", recording_filename);
        return 1;
    }

    if (record_imu)
    {
        CHECK(k4a_record_add_imu_track(recording), device);
    }
    CHECK(k4a_record_write_header(recording), device);

    // Wait for the first capture before starting recording.
    k4a_capture_t capture;
    seconds timeout_sec_for_first_capture(60);
    if (device_config->wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
    {
        timeout_sec_for_first_capture = seconds(360);
        LOGGER()->info("[subordinate mode] Waiting for signal from master");
    }
    steady_clock::time_point first_capture_start = steady_clock::now();
    k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
    // Wait for the first capture in a loop so Ctrl-C will still exit.
    while (!exiting && (steady_clock::now() - first_capture_start) < timeout_sec_for_first_capture)
    {
        result = k4a_device_get_capture(device, &capture, 100);
        if (result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_capture_release(capture);
            break;
        }
        else if (result == K4A_WAIT_RESULT_FAILED)
        {
            LOGGER()->error("K4ARecorder", "Runtime error: k4a_device_get_capture() returned error: %d", result);
            return 1;
        }
    }

    if (exiting)
    {
        k4a_device_close(device);
        return 0;
    }
    else if (result == K4A_WAIT_RESULT_TIMEOUT)
    {
        LOGGER()->error("K4ARecorder", "Timed out waiting for first capture.");
        return 1;
    }

    LOGGER()->info("Started recording");
    if (recording_length <= 0)
    {
        LOGGER()->info("Press Ctrl-C to stop recording.");
    }

    steady_clock::time_point recording_start = steady_clock::now();
    int32_t timeout_ms = 1000 / camera_fps;
    do
    {
        result = k4a_device_get_capture(device, &capture, timeout_ms);
        if (result == K4A_WAIT_RESULT_TIMEOUT)
        {
            continue;
        }
        else if (result != K4A_WAIT_RESULT_SUCCEEDED)
        {
            LOGGER()->error("K4ARecorder", "Runtime error: k4a_device_get_capture() returned error: %d", result);
            break;
        }
        CHECK(k4a_record_write_capture(recording, capture), device);
        k4a_capture_release(capture);

        if (record_imu)
        {
            do
            {
                k4a_imu_sample_t sample;
                result = k4a_device_get_imu_sample(device, &sample, 0);
                if (result == K4A_WAIT_RESULT_TIMEOUT)
                {
                    break;
                }
                else if (result != K4A_WAIT_RESULT_SUCCEEDED)
                {
                    LOGGER()->error("K4ARecorder", "Runtime error: k4a_imu_get_sample() returned %d", result);
                    break;
                }
                k4a_result_t write_result = k4a_record_write_imu_sample(recording, sample);
                if (K4A_FAILED(write_result))
                {
                    LOGGER()->error("K4ARecorder", "Runtime error: k4a_record_write_imu_sample() returned %d", write_result);
                    break;
                }
            } while (!exiting && result != K4A_WAIT_RESULT_FAILED &&
                     (recording_length < 0 || (steady_clock::now() - recording_start < recording_length_seconds)));
        }
        if (onFrameUpdate != NULL)
        {
            onFrameUpdate(state, send_frame_counter);
        }
    } while (!exiting && result != K4A_WAIT_RESULT_FAILED &&
             (recording_length < 0 || (steady_clock::now() - recording_start < recording_length_seconds)));

    if (!exiting)
    {
        exiting = true;
        LOGGER()->info("Stopping recording...");
    }

    if (record_imu)
    {
        k4a_device_stop_imu(device);
    }
    k4a_device_stop_cameras(device);

    LOGGER()->info("Saving recording...");
    CHECK(k4a_record_flush(recording), device);
    k4a_record_close(recording);

    LOGGER()->info("Done");

    k4a_device_close(device);

    return 0;
}
