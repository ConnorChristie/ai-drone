#include <thread>
#include <mutex>

#include "mongoose.h"
#include "blockingconcurrentqueue.h"

moodycamel::BlockingConcurrentQueue<cv::Mat> frame_queue;

static sig_atomic_t s_signal_received = 0;
static const char* s_http_port = "8000";
static struct mg_serve_http_opts s_http_server_opts;

static void signal_handler(int sig_num) {
    signal(sig_num, signal_handler);  // Reinstantiate signal handler
    s_signal_received = sig_num;
}

static int is_websocket(const struct mg_connection* nc) {
    return nc->flags & MG_F_IS_WEBSOCKET;
}

static void broadcast(struct mg_connection* nc, const struct mg_str msg) {
    struct mg_connection* c;
    char buf[500];
    char addr[32];
    mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
        MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);

    snprintf(buf, sizeof(buf), "%s %.*s", addr, (int)msg.len, msg.p);
    printf("%s\n", buf); /* Local echo. */
    for (c = mg_next(nc->mgr, NULL); c != NULL; c = mg_next(nc->mgr, c)) {
        if (c == nc) continue; /* Don't send to the sender. */
        mg_send_websocket_frame(c, WEBSOCKET_OP_TEXT, buf, strlen(buf));
    }
}

std::map<std::string, std::shared_ptr<mg_connection>> connections;
std::mutex connections_mutex;

static void ev_handler(struct mg_connection* nc, int ev, void* ev_data) {
    switch (ev) {
    case MG_EV_WEBSOCKET_HANDSHAKE_DONE: {

        char addr[32];
        mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
            MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);

        {
            std::lock_guard<std::mutex> lock(connections_mutex);
            connections.emplace(std::string(addr), std::shared_ptr<mg_connection>(nc));
        }

        break;
    }
    case MG_EV_WEBSOCKET_FRAME: {
        struct websocket_message* wm = (struct websocket_message*) ev_data;
        struct mg_str d = { (char*)wm->data, wm->size };

        break;
    }
    case MG_EV_HTTP_REQUEST: {
        mg_serve_http(nc, (struct http_message*) ev_data, s_http_server_opts);
        break;
    }
    case MG_EV_CLOSE: {
        char addr[32];
        mg_sock_addr_to_str(&nc->sa, addr, sizeof(addr),
            MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);

        {
            std::lock_guard<std::mutex> lock(connections_mutex);
            connections.erase(std::string(addr));
        }

        break;
    }
    }
}

void send_me()
{
    while (true)
    {
        cv::Mat frame;
        frame_queue.wait_dequeue(frame);

        vector<uchar> buffer;
        cv::imencode(".jpg", frame, buffer);

        {
            std::lock_guard<std::mutex> lock(connections_mutex);
            for (auto conn : connections)
            {
                mg_str msg = mg_mk_str("you get a frame!");
                char buf[500];

                snprintf(buf, sizeof(buf), "%.*s", (int)msg.len, msg.p);
                mg_send_websocket_frame(conn.second.get(), WEBSOCKET_OP_TEXT, buf, strlen(buf));
            }
        }

        cv::imshow("Detection results", frame);
        cv::waitKey(20);
    }
}

int run_web_server(void)
{
    struct mg_mgr mgr;
    struct mg_connection* nc;

    std::thread sender(&send_me);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mg_mgr_init(&mgr, NULL);

    nc = mg_bind(&mgr, s_http_port, ev_handler);
    mg_set_protocol_http_websocket(nc);

    printf("Started on port %s\n", s_http_port);
    while (s_signal_received == 0) {
        mg_mgr_poll(&mgr, 200);
    }
    mg_mgr_free(&mgr);

    sender.join();

    return 0;
}