#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace
{
cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromString(const std::string & name)
{
  static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> values = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
  };
  const auto iterator = values.find(name);
  if (iterator == values.end()) {
    throw std::invalid_argument("Unknown dictionary: " + name);
  }
  return iterator->second;
}

void printUsage(const char * program)
{
  std::cout
    << "Usage:\n  " << program
    << " OUTPUT.png [squares_x=7] [squares_y=5] [square_mm=30]"
    << " [marker_mm=22] [dictionary=DICT_5X5_100] [dpi=300] [margin_mm=10]\n\n"
    << "Example:\n  " << program
    << " /tmp/charuco.png 7 5 30 22 DICT_5X5_100 300 10\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  try {
    const std::string output_path = argv[1];
    const int squares_x = argc > 2 ? std::stoi(argv[2]) : 7;
    const int squares_y = argc > 3 ? std::stoi(argv[3]) : 5;
    const double square_mm = argc > 4 ? std::stod(argv[4]) : 30.0;
    const double marker_mm = argc > 5 ? std::stod(argv[5]) : 22.0;
    const std::string dictionary_name = argc > 6 ? argv[6] : "DICT_5X5_100";
    const double dpi = argc > 7 ? std::stod(argv[7]) : 300.0;
    const double margin_mm = argc > 8 ? std::stod(argv[8]) : 10.0;

    if (squares_x < 3 || squares_y < 3 || square_mm <= 0.0 ||
      marker_mm <= 0.0 || marker_mm >= square_mm || dpi <= 0.0 || margin_mm < 0.0)
    {
      throw std::invalid_argument("Invalid board dimensions");
    }

    const auto dictionary = cv::aruco::getPredefinedDictionary(
      dictionaryFromString(dictionary_name));
    const auto board = cv::aruco::CharucoBoard::create(
      squares_x, squares_y,
      static_cast<float>(square_mm),
      static_cast<float>(marker_mm),
      dictionary);

    const double pixels_per_mm = dpi / 25.4;
    const int board_width_px = static_cast<int>(std::lround(
      static_cast<double>(squares_x) * square_mm * pixels_per_mm));
    const int board_height_px = static_cast<int>(std::lround(
      static_cast<double>(squares_y) * square_mm * pixels_per_mm));
    const int margin_px = static_cast<int>(std::lround(margin_mm * pixels_per_mm));

    cv::Mat image;
    board->draw(
      cv::Size(board_width_px + 2 * margin_px, board_height_px + 2 * margin_px),
      image, margin_px, 1);

    if (!cv::imwrite(output_path, image)) {
      throw std::runtime_error("cv::imwrite returned false");
    }

    std::cout << "Generated: " << output_path << "\n"
              << "Board: " << squares_x << " x " << squares_y << " squares\n"
              << "Square: " << square_mm << " mm\n"
              << "Marker: " << marker_mm << " mm\n"
              << "Dictionary: " << dictionary_name << "\n"
              << "Nominal DPI: " << dpi << "\n"
              << "Print at 100% scale, then measure the real square and marker sizes.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << "\n";
    printUsage(argv[0]);
    return 1;
  }
}
