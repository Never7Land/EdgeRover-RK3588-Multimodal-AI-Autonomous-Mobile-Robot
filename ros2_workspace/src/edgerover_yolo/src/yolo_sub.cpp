#include <cstdio>
#include <zmq.h>

int main(int argc, char** argv) {
  const char* ep = argc > 1 ? argv[1] : "ipc:///tmp/edgerover_det";
  void* ctx = zmq_ctx_new();
  void* sub = zmq_socket(ctx, ZMQ_SUB);
  zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect(sub, ep);
  char buf[16384];
  while (true) {
    int n = zmq_recv(sub, buf, sizeof(buf) - 1, 0);
    if (n < 0) break;
    buf[n] = 0;
    printf("%s\n", buf);
    fflush(stdout);
  }
  return 0;
}