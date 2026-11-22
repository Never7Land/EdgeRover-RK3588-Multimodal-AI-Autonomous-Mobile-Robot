// vlm_ask: 命令行客户端，发一个问题给 vlm_proc 并打印回答
#include <chrono>
#include <cstdio>
#include <cstring>
#include <zmq.h>

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: vlm_ask \"question\" [endpoint=ipc:///tmp/edgerover_vlm]\n");
    return 1;
  }
  const char * endpoint = argc > 2 ? argv[2] : "ipc:///tmp/edgerover_vlm";

  void * ctx = zmq_ctx_new();
  void * s = zmq_socket(ctx, ZMQ_REQ);
  int timeout_ms = 120000, linger = 0;
  zmq_setsockopt(s, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));  // 服务端没起时不至于永久卡住
  zmq_setsockopt(s, ZMQ_LINGER, &linger, sizeof(linger));
  zmq_connect(s, endpoint);

  auto t0 = std::chrono::steady_clock::now();
  zmq_send(s, argv[1], std::strlen(argv[1]), 0);

  zmq_msg_t m;
  zmq_msg_init(&m);
  const int n = zmq_msg_recv(&m, s, 0);
  int rc = 0;
  if (n < 0) {
    std::fprintf(stderr, "no reply: %s (vlm_proc running?)\n", zmq_strerror(errno));
    rc = 1;
  } else {
    std::fwrite(zmq_msg_data(&m), 1, n, stdout);
    std::printf("\n[client] round trip %ld ms\n", (long)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count());
  }
  zmq_msg_close(&m);
  zmq_close(s);
  zmq_ctx_term(ctx);
  return rc;
}