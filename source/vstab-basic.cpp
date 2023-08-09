// #include "precomp.h"

///////////////////////////////////////////////////////////////////////////////
// 类型定义

// 轨迹 图形变换的关键参数 平移和旋转
struct Trajectory {
  Trajectory() = default;
  Trajectory(double x, double y, double a) : x(x), y(y), a(a) {}

  friend Trajectory operator+(const Trajectory& c1, const Trajectory& c2) {
    return {c1.x + c2.x, c1.y + c2.y, c1.a + c2.a};
  }
  friend Trajectory operator-(const Trajectory& c1, const Trajectory& c2) {
    return {c1.x - c2.x, c1.y - c2.y, c1.a - c2.a};
  }
  friend Trajectory operator*(const Trajectory& c1, const Trajectory& c2) {
    return {c1.x * c2.x, c1.y * c2.y, c1.a * c2.a};
  }
  friend Trajectory operator/(const Trajectory& c1, const Trajectory& c2) {
    return {c1.x / c2.x, c1.y / c2.y, c1.a / c2.a};
  }

  static Trajectory one() { return {1, 1, 1}; }
  static Trajectory zero() { return {0, 0, 0}; }
  static Trajectory all(double v) { return {v, v, v}; }

  double x;
  double y;
  double a;
};

using GoodMatchCorners = std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>;
using GoodMatchCornersConfig = std::tuple<int, double, double>;

///////////////////////////////////////////////////////////////////////////////
// 匹配特征点

GoodMatchCorners findGoodMatchCorners(const cv::Mat& prevImage,
                                      const cv::Mat& nextImage,
                                      const GoodMatchCornersConfig& config) {
  assert(prevImage.type() == CV_8UC1 && nextImage.type() == CV_8UC1);

  auto&& [maxCorners, qualityLevel, minDistance] = config;

  std::vector<cv::Point2f> prevCorners;
  cv::goodFeaturesToTrack(prevImage, prevCorners, maxCorners, qualityLevel, minDistance);

  std::vector<cv::Point2f> nextCorners;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prevImage, nextImage, prevCorners, nextCorners, status, err);

  // TODO(zixianwei): 希望找到一种高效的移除未能匹配的点对
  std::vector<cv::Point2f> prevCorners2;
  prevCorners2.reserve(prevCorners.size());
  std::vector<cv::Point2f> nextCorners2;
  nextCorners2.reserve(nextCorners.size());
  for (size_t i = 0; i < status.size(); i++) {
    if (status[i] != 0U) {
      prevCorners2.emplace_back(prevCorners[i]);
      nextCorners2.emplace_back(nextCorners[i]);
    }
  }

  return {prevCorners2, nextCorners2};
}

///////////////////////////////////////////////////////////////////////////////
// 变换矩阵-卡尔曼滤波-变换矩阵

void filtAffineTransform(const std::vector<cv::Point2f>& prevCorners,
                         const std::vector<cv::Point2f>& nextCorners,
                         cv::Mat& matrix) {
  cv::Mat transform = cv::estimateAffinePartial2D(prevCorners, nextCorners);
  if (!transform.empty()) {
    // 静态局部变量 在第一次访问时被初始化
    static Trajectory X = Trajectory::zero();
    static Trajectory P = Trajectory::one();
    static Trajectory Q = Trajectory::all(4e-3);
    static Trajectory R = Trajectory::all(0.25);

    static double x = 0;
    static double y = 0;
    static double a = 0;

    // 只考虑平移和旋转
    double dx = transform.at<double>(0, 2);
    double dy = transform.at<double>(1, 2);
    double da = std::atan2(transform.at<double>(1, 0), transform.at<double>(0, 0));

    x += dx;
    y += dy;
    a += da;

    Trajectory z = Trajectory(x, y, a);
    Trajectory X_ = X;
    Trajectory P_ = P + Q;
    Trajectory K = P_ / (P_ + R);
    X = X_ + K * (z - X_);
    P = (Trajectory::one() - K) * P_;

    dx += X.x - x;
    dy += X.y - y;
    da += X.a - a;

    matrix.at<double>(0, 0) = cos(da);
    matrix.at<double>(0, 1) = -sin(da);
    matrix.at<double>(1, 0) = sin(da);
    matrix.at<double>(1, 1) = cos(da);

    matrix.at<double>(0, 2) = dx;
    matrix.at<double>(1, 2) = dy;
  }
}

///////////////////////////////////////////////////////////////////////////////
// 图像逆变换

cv::Mat getNextByApplyAffine(const cv::Mat& prevImage, const cv::Mat& matrix) {
  cv::Mat nextImage;
  cv::warpAffine(prevImage, nextImage, matrix, prevImage.size());
  return nextImage;
}

///////////////////////////////////////////////////////////////////////////////
// 参数解析

std::tuple<std::string, std::string, GoodMatchCornersConfig> parseArguments(const std::string& configPath) {
  std::string inputPath;
  std::string outputPath;
  int goodMatchCornersMaxCorners = 200;
  double goodMatchCornersQualityLevel = 0.01;
  double goodMatchCornersMinDistance = 30.0;
  bool affineTransformFullAffine = false;

  std::ifstream configFile(configPath);
  if (!configFile.is_open()) {
    std::cout << fmt::format("Cannot read config file: [{}]\n", configPath);
    exit(EXIT_FAILURE);
  }

  std::string configString((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());

  rapidjson::Document configJson;
  configJson.Parse(configString.c_str());
  if (configJson.HasParseError()) {
    std::cout << fmt::format("Cannot parse config file: [{}]\n");
    exit(EXIT_FAILURE);
  }

  if (configJson.HasMember("inputPath") && configJson["inputPath"].IsString()) {
    inputPath = configJson["inputPath"].GetString();
  } else {
    std::cout << fmt::format("Config file cannot get member: [inputPath]\n");
    exit(EXIT_FAILURE);
  }

  if (configJson.HasMember("outputPath") && configJson["outputPath"].IsString()) {
    outputPath = configJson["outputPath"].GetString();
  } else {
    std::cout << fmt::format("Config file cannot get member: [outputPath]\n");
    exit(EXIT_FAILURE);
  }

  if (configJson.HasMember("goodMatchCorners") && configJson["goodMatchCorners"].IsObject()) {
    const rapidjson::Value& object = configJson["goodMatchCorners"];
    if (object.HasMember("maxCorners") && object["maxCorners"].IsInt()) {
      goodMatchCornersMaxCorners = object["maxCorners"].GetInt();
    } else {
      std::cout << fmt::format("Config file cannot get member: [goodMatchCorners - maxCorners]\n");
      exit(EXIT_FAILURE);
    }
    if (object.HasMember("qualityLevel") && object["qualityLevel"].IsDouble()) {
      goodMatchCornersQualityLevel = object["qualityLevel"].GetDouble();
    } else {
      std::cout << fmt::format("Config file cannot get member: [goodMatchCorners - qualityLevel]\n");
      exit(EXIT_FAILURE);
    }
    if (object.HasMember("minDistance") && object["minDistance"].IsDouble()) {
      goodMatchCornersMinDistance = object["minDistance"].GetDouble();
    } else {
      std::cout << fmt::format("Config file cannot get member: [goodMatchCorners - minDistance]\n");
      exit(EXIT_FAILURE);
    }
  } else {
    std::cout << fmt::format("Config file cannot get member: [goodMatchCorners]\n");
    exit(EXIT_FAILURE);
  }

  GoodMatchCornersConfig goodMatchCornersConfig{goodMatchCornersMaxCorners, goodMatchCornersQualityLevel,
                                                goodMatchCornersMinDistance};

  return std::make_tuple(inputPath, outputPath, goodMatchCornersConfig);
}

///////////////////////////////////////////////////////////////////////////////
// 主函数

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: ./vstab-basic <path_to_config.json>\n";
    exit(EXIT_FAILURE);
  }
  std::string configPath = std::string(argv[1]);

  std::string inputPath;
  std::string outputPath;
  GoodMatchCornersConfig goodMatchCornersConfig;
  std::tie(inputPath, outputPath, goodMatchCornersConfig) = parseArguments(configPath);

  cv::VideoCapture inputVideo(inputPath);
  if (!inputVideo.isOpened()) {
    std::cout << fmt::format("Cannot open input video. inputPath = [{}]\n", inputPath);
    exit(EXIT_FAILURE);
  }

  int frameWidth = inputVideo.get(cv::CAP_PROP_FRAME_WIDTH);
  int frameHeight = inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT);
  int frameCount = inputVideo.get(cv::CAP_PROP_FRAME_COUNT);
  double fps = inputVideo.get(cv::CAP_PROP_FPS);

  cv::VideoWriter outputVideo;
  outputVideo.open(outputPath, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(frameWidth, frameHeight));
  if (!outputVideo.isOpened()) {
    std::cout << fmt::format("Cannot open output video. outputPath = [{}]\n", outputPath);
    exit(EXIT_FAILURE);
  }

  cv::Mat prevImage;
  cv::Mat prevImageGray;
  cv::Mat nextImage;
  cv::Mat nextImageGray;

  inputVideo.read(prevImage);
  cv::cvtColor(prevImage, prevImageGray, cv::COLOR_BGR2GRAY);

  cv::Mat matrix = cv::Mat::zeros(2, 3, CV_64F);

  cv::TickMeter tm;
  int kCount = 1;

  while (true) {
    tm.start();

    // 读取下一帧图像
    inputVideo.read(nextImage);
    if (nextImage.empty()) {
      break;
    }
    cv::cvtColor(nextImage, nextImageGray, cv::COLOR_BGR2GRAY);

    // 搜索前一帧和后一帧中能够良好匹配的特征点
    auto [prevCorners, nextCorners] = findGoodMatchCorners(prevImageGray, nextImageGray, goodMatchCornersConfig);

    // 利用特征点对估计变换矩阵 并使用卡尔曼滤波平滑变换矩阵
    filtAffineTransform(prevCorners, nextCorners, matrix);

    // 对前一帧应用平滑后的变换矩阵 得到后一帧的最佳估计
    cv::Mat nextImageFiltered = getNextByApplyAffine(prevImage, matrix);

    tm.stop();

    // 逐帧保存视频流
    outputVideo.write(nextImageFiltered);

    // 将后一帧数据保存到前一帧 这里利用了|cv::Mat|的引用计数特性
    prevImage = nextImage;
    prevImageGray = nextImageGray;
    nextImage.release();
    nextImageGray.release();

    // 在终端中打印一些辅助信息
    // std::cout << fmt::format("Frame: [{}/{}] - GFTT: [{}/{}]\n", kCount,
    //                          frameCount, prevCorners.size(),
    //                          nextCorners.size());
    kCount++;
  }

  std::cout << fmt::format("TM: [{:.6f}] - [{:.6f}]\n", tm.getAvgTimeMilli(), tm.getTimeMilli());

  return 0;
}
