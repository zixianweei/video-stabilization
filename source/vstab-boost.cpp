// #include "precomp.h"

///////////////////////////////////////////////////////////////////////////////
// 类型定义

// 帧标签 保证有序性
class FrameTag {
 public:
  FrameTag() = default;
  FrameTag(uint16_t id, uint16_t gid) : id_(id), gid_(gid) {}
  [[nodiscard]] uint16_t id() const { return id_; }
  [[nodiscard]] uint16_t gid() const { return gid_; }
  // [[nodiscard]] uint16_t next() const { return (id_ + 1) % kMod; }
  void add() {
    if (id_ == kMod - 1) {
      gid_ = (gid_ + 1) % kMod;
    }
    id_ = (id_ + 1) % kMod;
  }
  bool operator<(const FrameTag& rhs) const {
    if (gid_ == rhs.gid_) {
      return id_ > rhs.id();
    }
    return gid_ > rhs.gid_;
  }

 private:
  static constexpr uint16_t kMod = std::numeric_limits<uint16_t>::max();
  uint16_t id_{0};
  uint16_t gid_{};
};

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

// 变量的别名
using GoodMatchCorners = std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>;
using GoodMatchCornersConfig = std::tuple<int, double, double>;

using TaggedImage = std::pair<FrameTag, std::tuple<cv::Mat, cv::Mat, cv::Mat>>;
using TaggedGoodMatchCorners = std::pair<FrameTag, std::pair<cv::Mat, GoodMatchCorners>>;
using TaggedAffineTransform = std::pair<FrameTag, std::pair<cv::Mat, cv::Mat>>;
using TaggedApplyAffine = std::pair<FrameTag, cv::Mat>;

#define PriorityQueue(Tp, Fc) std::priority_queue<Tp, std::vector<Tp>, decltype(Fc)>

using AtomicBool = std::atomic_bool;
using Mutex = std::mutex;
using ConditionVariable = std::condition_variable;

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
      std::cout << fmt::format(
          "Config file cannot get member: [goodMatchCorners - "
          "qualityLevel]\n");
      exit(EXIT_FAILURE);
    }
    if (object.HasMember("minDistance") && object["minDistance"].IsDouble()) {
      goodMatchCornersMinDistance = object["minDistance"].GetDouble();
    } else {
      std::cout << fmt::format(
          "Config file cannot get member: [goodMatchCorners - "
          "minDistance]\n");
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
    std::cout << "Usage: ./vstab-boost <path_to_config.json>\n";
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

  /// 表示当前线程状态
  AtomicBool isReadImageExit;
  isReadImageExit.store(false);
  AtomicBool isGoodMatchCornersExit;
  isGoodMatchCornersExit.store(false);
  AtomicBool isAffineTransformExit;
  isAffineTransformExit.store(false);
  AtomicBool isApplyAffineExit;
  isApplyAffineExit.store(false);
  AtomicBool isWriteImageExit;
  isWriteImageExit.store(false);

  auto frameTagCompare = [](const auto& f1, const auto& f2) { return f1.first < f2.first; };

  PriorityQueue(TaggedImage, frameTagCompare) readImagePQ(frameTagCompare);
  Mutex mtxReadImagePQ;
  ConditionVariable cvReadImagePQ;

  PriorityQueue(TaggedGoodMatchCorners, frameTagCompare) goodMatchCornersPQ(frameTagCompare);
  Mutex mtxGoodMatchCornersPQ;
  ConditionVariable cvGoodMatchCornersPQ;

  PriorityQueue(TaggedAffineTransform, frameTagCompare) affineTransformPQ(frameTagCompare);
  Mutex mtxAffineTransformPQ;
  ConditionVariable cvAffineTransformPQ;

  PriorityQueue(TaggedApplyAffine, frameTagCompare) applyAffinePQ(frameTagCompare);
  Mutex mtxApplyAffinePQ;
  ConditionVariable cvApplyAffinePQ;

  Mutex mtxWriteImageExit;
  ConditionVariable cvWriteImageExit;

  auto funcReadImage = [&] {
    cv::Mat prevImage;
    cv::Mat prevImageGray;
    cv::Mat nextImage;
    cv::Mat nextImageGray;

    inputVideo.read(prevImage);
    cv::cvtColor(prevImage, prevImageGray, cv::COLOR_BGR2GRAY);

    thread_local FrameTag frameTag;
    while (true) {
      inputVideo.read(nextImage);
      if (nextImage.empty()) {
        isReadImageExit.store(true);
        break;
      }
      cv::cvtColor(nextImage, nextImageGray, cv::COLOR_BGR2GRAY);

      std::unique_lock<std::mutex> lock(mtxReadImagePQ);

      readImagePQ.emplace(frameTag, std::make_tuple(prevImage, prevImageGray, nextImageGray));
      frameTag.add();

      lock.unlock();
      cvReadImagePQ.notify_one();

      prevImage = nextImage;
      prevImageGray = nextImageGray;
      nextImage.release();
      nextImageGray.release();
    }

    std::cout << fmt::format("funcReadImage exit... - [{}]\n", frameTag.id());
    cvReadImagePQ.notify_one();
  };

  auto funcGoodMatchCorners = [&] {
    thread_local FrameTag frameTag;
    while (true) {
      std::unique_lock lockReadImagePQ(mtxReadImagePQ);
      cvReadImagePQ.wait(lockReadImagePQ, [&readImagePQ, &isReadImageExit] {
        if (readImagePQ.empty()) {
          return isReadImageExit.load();
        }
        return true;
      });

      if (readImagePQ.empty() && isReadImageExit.load()) {
        isGoodMatchCornersExit.store(true);
        break;
      }

      auto [tag, tuple] = readImagePQ.top();
      readImagePQ.pop();
      frameTag.add();

      lockReadImagePQ.unlock();

      auto&& [prevImage, prevImageGray, nextImageGray] = tuple;
      auto&& goodMatchCorners = findGoodMatchCorners(prevImageGray, nextImageGray, goodMatchCornersConfig);

      std::unique_lock lockGoodMatchCornersPQ(mtxGoodMatchCornersPQ);

      goodMatchCornersPQ.emplace(tag, std::make_pair(prevImage.clone(), goodMatchCorners));

      lockGoodMatchCornersPQ.unlock();
      cvGoodMatchCornersPQ.notify_one();
    }

    std::cout << fmt::format("funcGoodMatchCorners exit... - [{}]\n", frameTag.id());
    cvGoodMatchCornersPQ.notify_one();
  };

  auto funcAffineTransform = [&] {
    thread_local FrameTag frameTag;
    thread_local cv::Mat matrix = cv::Mat::zeros(2, 3, CV_64F);
    while (true) {
      std::unique_lock lockGoodMatchCornersPQ(mtxGoodMatchCornersPQ);
      cvGoodMatchCornersPQ.wait(lockGoodMatchCornersPQ, [&goodMatchCornersPQ, &isGoodMatchCornersExit] {
        if (goodMatchCornersPQ.empty()) {
          return isGoodMatchCornersExit.load();
        }
        return true;
      });

      if (goodMatchCornersPQ.empty() && isGoodMatchCornersExit.load()) {
        isAffineTransformExit.store(true);
        break;
      }

      auto [tag, pair] = goodMatchCornersPQ.top();
      goodMatchCornersPQ.pop();
      frameTag.add();

      lockGoodMatchCornersPQ.unlock();

      auto&& [prevImage, goodMatchCorners] = pair;
      auto&& [prevCorners, nextCorners] = goodMatchCorners;

      filtAffineTransform(prevCorners, nextCorners, matrix);

      std::unique_lock lockAffineTransformPQ(mtxAffineTransformPQ);

      affineTransformPQ.emplace(tag, std::make_pair(prevImage.clone(), matrix.clone()));

      lockAffineTransformPQ.unlock();
      cvAffineTransformPQ.notify_one();
    }

    std::cout << fmt::format("funcAffineTransform exit... - [{}]\n", frameTag.id());
    cvAffineTransformPQ.notify_one();
  };

  auto funcApplyAffine = [&] {
    thread_local FrameTag frameTag;
    while (true) {
      std::unique_lock lockAffineTransformPQ(mtxAffineTransformPQ);
      cvAffineTransformPQ.wait(lockAffineTransformPQ, [&affineTransformPQ, &isAffineTransformExit] {
        if (affineTransformPQ.empty()) {
          return isAffineTransformExit.load();
        }
        return true;
      });

      if (affineTransformPQ.empty() && isAffineTransformExit.load()) {
        isApplyAffineExit.store(true);
        break;
      }

      auto [tag, pair] = affineTransformPQ.top();
      affineTransformPQ.pop();
      frameTag.add();

      lockAffineTransformPQ.unlock();

      auto&& [prevImage, affineTransform] = pair;
      cv::Mat appliedImage = getNextByApplyAffine(prevImage, affineTransform);

      std::unique_lock lockApplyAffine(mtxApplyAffinePQ);

      applyAffinePQ.emplace(tag, appliedImage.clone());

      lockApplyAffine.unlock();
      cvApplyAffinePQ.notify_one();
    }

    std::cout << fmt::format("funcApplyAffine exit... - [{}]\n", frameTag.id());
    cvApplyAffinePQ.notify_one();
  };

  auto funcWriteImage = [&] {
    thread_local FrameTag frameTag;
    while (true) {
      std::unique_lock lockApplyAffine(mtxApplyAffinePQ);
      cvApplyAffinePQ.wait(lockApplyAffine, [&applyAffinePQ, &isApplyAffineExit] {
        if (applyAffinePQ.empty()) {
          return isApplyAffineExit.load();
        }
        return true;
      });

      if (applyAffinePQ.empty() && isApplyAffineExit.load()) {
        isWriteImageExit.store(true);
        break;
      }

      auto [tag, image] = applyAffinePQ.top();
      applyAffinePQ.pop();
      frameTag.add();

      lockApplyAffine.unlock();

      outputVideo.write(image);
      // std::cout << fmt::format("[{}/{}]\n", frameTag.id(), frameCount);
    }

    std::cout << fmt::format("funcWriteImage exit... - [{}]\n", frameTag.id());
    cvWriteImageExit.notify_one();
  };

  std::thread thReadImage(funcReadImage);
  std::thread thGoodMatchCorners(funcGoodMatchCorners);
  std::thread thAffineTransform(funcAffineTransform);
  std::thread thApplyAffine(funcApplyAffine);
  std::thread thWriteImage(funcWriteImage);

  std::unique_lock lockWriteImageExit(mtxWriteImageExit);
  cvWriteImageExit.wait(lockWriteImageExit, [&isWriteImageExit] { return isWriteImageExit.load(); });

  thReadImage.join();
  thGoodMatchCorners.join();
  thAffineTransform.join();
  thApplyAffine.join();
  thWriteImage.join();

  std::cout << "exit...\n";

  return EXIT_SUCCESS;
}
