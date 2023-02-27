#define BOOST_TEST_MODULE orb_grpc_server tests
#include "orbslam_server_v1.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/test/included/unit_test.hpp>
#include <exception>
#include <iostream>
namespace fs = boost::filesystem;

namespace viam {
namespace {

void checkParseAndValidateArgumentsException(const vector<string>& args,
                                             const string& message) {
    SLAMServiceImpl slamService;
    BOOST_CHECK_EXCEPTION(utils::ParseAndValidateArguments(args, slamService),
                          runtime_error, [&message](const runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_no_args) {
    const vector<string> args;
    const string message =
        "No args found. Expected: \n"
        "./bin/orb_grpc_server "
        "-data_dir=path_to_data "
        "-config_param={mode=slam_mode,} "
        "-port=grpc_port "
        "-sensors=sensor_name "
        "-data_rate_ms=frame_delay "
        "-map_rate_sec=map_rate_sec "
        "-delete_processed_data=delete_data "
        "-use_live_data=offline_or_online";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_no_data_dir) {
    const vector<string> args{"-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=true",
                              "-unknown=unknown"};
    const string message = "No data directory given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_no_slam_mode) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=true"};
    const string message = "No SLAM mode given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_invalid_slam_mode) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=bad}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=true"};
    const string message = "Invalid slam_mode=bad";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_no_slam_port) {
    const vector<string> args{
        "-data_dir=/path/to",  "-config_param={mode=rgbd}",
        "-sensors=color",      "-data_rate_ms=200",
        "-map_rate_sec=60",    "-delete_processed_data=false",
        "-use_live_data=true", "-unknown=unknown"};
    const string message = "No gRPC port given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_valid_config_no_data_rate_msec) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=true"};
    const string message = "a data_rate_ms value is required";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_valid_config_no_map_rate_sec) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=",
                              "-delete_processed_data=false",
                              "-use_live_data=true"};
    const string message = "a map_rate_sec value is required";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_valid_config) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=true"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.path_to_vocab == "/path/to/config/ORBvoc.txt");
    BOOST_TEST(slamService.path_to_settings == "/path/to/config");
    BOOST_TEST(slamService.path_to_data == "/path/to/data");
    BOOST_TEST(slamService.path_to_map == "/path/to/map");
    BOOST_TEST(slamService.slam_mode == "rgbd");
    BOOST_TEST(slamService.slam_port == "20000");
    BOOST_TEST(slamService.frame_delay_msec.count() ==
               chrono::milliseconds(200).count());
    BOOST_TEST(slamService.map_rate_sec.count() == chrono::seconds(60).count());
    BOOST_TEST(slamService.camera_name == "color");
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.delete_processed_data == false);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_valid_config_capitalized_slam_mode) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=RGBD}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=true",
                              "-use_live_data=true"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.slam_mode == "rgbd");
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_valid_config_no_camera) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=false"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.camera_name == "");
    BOOST_TEST(slamService.use_live_data == false);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_online_with_true_delete_processed_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=true",
                              "-use_live_data=true"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.delete_processed_data == true);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_online_with_false_delete_processed_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=true"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.delete_processed_data == false);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_offline_with_true_delete_processed_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=true",
                              "-use_live_data=false"};
    const string message =
        "a true delete_processed_data value is invalid when running slam in "
        "offline mode";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_offline_with_false_delete_processed_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=false"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.use_live_data == false);
    BOOST_TEST(slamService.delete_processed_data == false);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_invalid_delete_processed_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=gibberish"
                              "-use_live_data=true"};
    const string message =
        "invalid delete_processed_data value, set to either true or false";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_config_with_true_use_live_data_and_sensors) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=true",
                              "-use_live_data=true"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.camera_name == "color");
    BOOST_TEST(slamService.use_live_data == true);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_config_with_false_use_live_data_and_sensors) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=false"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.camera_name == "color");
    BOOST_TEST(slamService.use_live_data == false);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_config_with_true_use_live_data_and_no_sensors) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=true",
                              "-use_live_data=true"};
    const string message =
        "a true use_live_data value is invalid when no sensors are given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateArguments_config_with_false_use_live_data_and_no_sensors) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=false"};
    SLAMServiceImpl slamService;
    utils::ParseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.camera_name == "");
    BOOST_TEST(slamService.use_live_data == false);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_config_invalid_use_live_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data=gibberish"};
    const string message =
        "invalid use_live_data value, set to either true or false";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ParseAndValidateArguments_config_no_use_live_data) {
    const vector<string> args{"-data_dir=/path/to",
                              "-config_param={mode=rgbd}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-delete_processed_data=false",
                              "-use_live_data="};
    const string message =
        "invalid use_live_data value, set to either true or false";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(ReadTimeFromTimestamp_missing_timestamp) {
    // Provide a filename with a missing timestamp
    std::string timestamp = "no-timestamp";
    const std::string message =
        "timestamp cannot be parsed into a std::tm object: " + timestamp;
    BOOST_CHECK_EXCEPTION(utils::ReadTimeFromTimestamp(timestamp),
                          std::runtime_error,
                          [&message](const std::runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });
}

BOOST_AUTO_TEST_CASE(ReadTimeFromTimestamp) {
    const string timestamp_1 = "2022-01-01T01:00:00.0000Z";
    const string timestamp_2 = "2022-01-01T01:00:00.0001Z";
    const string timestamp_3 = "2022-01-01T01:00:01.0000Z";
    const auto time_1 = utils::ReadTimeFromTimestamp(timestamp_1);
    const auto time_2 = utils::ReadTimeFromTimestamp(timestamp_2);
    const auto time_3 = utils::ReadTimeFromTimestamp(timestamp_3);
    BOOST_TEST(time_1 < time_2);
    BOOST_TEST(time_2 < time_3);
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Closest_no_files) {
    const string configTimeString = "2022-01-01T01:00:00.0000Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files;
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Closest,
                                     configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Closest_ignore_last) {
    const string configTimeString = "2022-01-01T01:00:00.0001Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z"};
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Closest,
                                     configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Closest_found_time) {
    const string configTimeString = "2022-01-01T01:00:00.0000Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z",
                         "color_data_2022-01-01T01:00:00.0003Z"};
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Closest,
                                     configTime, &timeInterest) == 1);
    BOOST_TEST(timeInterest ==
               utils::ReadTimeFromTimestamp("2022-01-01T01:00:00.0001Z"));
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Recent_no_files) {
    const string configTimeString = "2022-01-01T01:00:00.0000Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files;
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Recent,
                                     configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Recent_ignore_last_mono) {
    const string configTimeString = "2022-01-01T01:00:00.0000Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z"};
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Recent,
                                     configTime, &timeInterest) == 1);
    BOOST_TEST(timeInterest ==
               utils::ReadTimeFromTimestamp("2022-01-01T01:00:00.0001Z"));
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Recent_ignore_last_mono_fail) {
    const string configTimeString = "2022-01-01T01:00:00.0002Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z"};
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Recent,
                                     configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Recent_ignore_last_rgbd_fail) {
    const string configTimeString = "2022-01-01T01:00:00.0002Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z"};
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "rgbd", "",
                                     utils::FileParserMethod::Recent,
                                     configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Recent_found_mono) {
    const string configTimeString = "2022-01-01T01:00:00.0000Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z",
                         "color_data_2022-01-01T01:00:00.0003Z",
                         "color_data_2022-01-01T01:00:00.0004Z"};
    double timeInterest;
    BOOST_TEST(utils::FindFrameIndex(files, "mono", "",
                                     utils::FileParserMethod::Recent,
                                     configTime, &timeInterest) == 3);
    BOOST_TEST(timeInterest ==
               utils::ReadTimeFromTimestamp("2022-01-01T01:00:00.0003Z"));
}

BOOST_AUTO_TEST_CASE(FindFrameIndex_Recent_found_time_rgbd) {
    const string configTimeString = "2022-01-01T01:00:00.0000Z";
    const auto configTime = utils::ReadTimeFromTimestamp(configTimeString);
    vector<string> files{"color_data_2022-01-01T01:00:00.0000Z",
                         "color_data_2022-01-01T01:00:00.0001Z",
                         "color_data_2022-01-01T01:00:00.0002Z",
                         "color_data_2022-01-01T01:00:00.0003Z"};
    double timeInterest;
    // Create a unique path in the temp directory
    fs::path tmp_dir = fs::temp_directory_path() / fs::unique_path();
    bool ok = fs::create_directory(tmp_dir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " +
                                 tmp_dir.string());
    }
    // Create the "depth" subdirectory
    fs::path tmp_dir_depth = tmp_dir / "depth";
    ok = fs::create_directory(tmp_dir_depth);
    if (!ok) {
        fs::remove_all(tmp_dir);
        throw std::runtime_error("could not create directory: " +
                                 tmp_dir_depth.string());
    }

    // Create the file in the temporary directory
    fs::ofstream ofs(tmp_dir_depth /
                     "color_data_2022-01-01T01:00:00.0001Z.png");
    ofs.close();
    BOOST_TEST(utils::FindFrameIndex(files, "rgbd", tmp_dir.string(),
                                     utils::FileParserMethod::Recent,
                                     configTime, &timeInterest) == 1);
    BOOST_TEST(timeInterest ==
               utils::ReadTimeFromTimestamp("2022-01-01T01:00:00.0001Z"));
    // Close the file and remove the temporary directory and its contents.
    fs::remove_all(tmp_dir);
}

}  // namespace
}  // namespace viam
