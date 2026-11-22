// 全屏摄像头预览：读 FrameRing（camera_proc 的输出）铺满屏幕，Qt EGLFS 直接画到 DRM，不需要桌面
#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

#include <QApplication>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QScreen>
#include <QTimer>

#include "edgerover_ipc/frame_ring.hpp"

class View : public QLabel
{
public:
  View()
  {
    setStyleSheet("background-color:black;");   // 黑底：画面比屏幕小时四周是黑边而不是白屏
    setWindowFlags(Qt::Window);                 // 去掉标题栏
    setAlignment(Qt::AlignCenter);
    setScaledContents(false);
    showFullScreen();                           // 直接全屏，无边框

    // 640x640 是正方形，缩放到屏幕短边即可，长边两边留黑
    if (QScreen * s = screen()) {
      const QSize g = s->geometry().size();
      const int m = std::min(g.width(), g.height());
      target_ = QSize(m, m);
    }

    open_ring();

    timer_ = new QTimer(this);
    // 30 Hz 轮询：FrameRing 里只有最新帧，读不到新帧就跳过这一轮，不会积压
    connect(timer_, &QTimer::timeout, [this]() {tick();});
    timer_->start(33);
  }

private:
  // camera_proc 可能还没起来，打不开共享内存就每秒重试一次
  void open_ring()
  {
    try {
      reader_ = std::make_unique<edgerover::ipc::FrameRingReader>("/edgerover_frames");
      buf_.resize(reader_->slot_size());
      std::printf("[camera_view] shm opened, slot=%u\n", reader_->slot_size());
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[camera_view] open shm failed: %s, retry in 1 s\n", e.what());
      QTimer::singleShot(1000, this, [this]() {open_ring();});
    }
  }

  void tick()
  {
    if (!reader_) {return;}
    edgerover::ipc::FrameInfo info{};
    const auto r = reader_->read_latest(buf_.data(), buf_.size(), info, last_id_);
    // 只有读到完整新帧才画；撕裂帧（kTorn）和无新帧都直接跳过
    if (r != edgerover::ipc::ReadResult::kOk || info.width == 0) {return;}
    last_id_ = info.frame_id;

    // 只引用共享内存缓冲区；scaled() 会生成自己拥有数据的新 QImage，之后才去碰 buf_
    const QImage src(buf_.data(), info.width, info.height, QImage::Format_RGB888);
    // 最近邻缩放，CPU 开销最小；摄像头装反时把下面这行换成 src.mirrored(true, false)
    const QImage shown = src.scaled(target_, Qt::KeepAspectRatio, Qt::FastTransformation);
    setPixmap(QPixmap::fromImage(shown));
  }

  std::unique_ptr<edgerover::ipc::FrameRingReader> reader_;
  std::vector<uint8_t> buf_;
  uint64_t last_id_{0};
  QSize target_{640, 640};
  QTimer * timer_{nullptr};
};

int main(int argc, char ** argv)
{
  // 没有桌面时默认走 eglfs；如果板子上 EGL 不可用，启动时手动改 QT_QPA_PLATFORM=linuxfb
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
    qputenv("QT_QPA_PLATFORM", "eglfs");
  }
  QApplication app(argc, argv);
  View v;
  return app.exec();
}