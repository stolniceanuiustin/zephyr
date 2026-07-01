#include "fb_udp.h"
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

LOG_MODULE_REGISTER(fb_udp, LOG_LEVEL_INF);

/* TCP server: the board listens, the PC viewer connects in. TCP gives ordered,
 * reliable, flow-controlled delivery, so unlike the old UDP scheme there are no
 * dropped chunks / discarded frames and the emulator is naturally paced by the
 * link instead of overflowing the TX buffer pool. */
#define LISTEN_PORT  5555

/* Wire format per frame: 4-byte big-endian length, then that many raw bytes
 * (the ARGB32 framebuffer). The viewer reads the length, then that many bytes. */
#define LEN_HEADER_SIZE 4

static int listen_sock = -1;
static int conn_sock = -1;

/* Wait for the default interface to come up and get an IPv4 address. The
 * emulator threads start almost immediately, but GEM link/static address only
 * settle a few seconds into boot. */
static int wait_for_iface(void)
{
    struct net_if *iface = NULL;

    LOG_INF("waiting for network interface to be ready...");
    for (int i = 0; i < 400; i++) {            /* up to ~20s */
        if (iface == NULL) {
            iface = net_if_get_default();
        }
        if (iface != NULL && net_if_is_up(iface) &&
            net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL) {
            break;
        }
        k_msleep(50);
    }

    if (iface == NULL ||
        net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == NULL) {
        LOG_ERR("network interface not ready after timeout");
        return -ETIMEDOUT;
    }
    LOG_INF("network interface ready");
    return 0;
}

int fb_udp_init(void)
{
    int ret = wait_for_iface();
    if (ret < 0) {
        return ret;
    }

    listen_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        LOG_ERR("socket() failed: %d", errno);
        return -errno;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(LISTEN_PORT);

    if (zsock_bind(listen_sock, (struct sockaddr *)&bind_addr,
                   sizeof(bind_addr)) < 0) {
        LOG_ERR("bind() failed: %d", errno);
        zsock_close(listen_sock);
        listen_sock = -1;
        return -errno;
    }

    if (zsock_listen(listen_sock, 1) < 0) {
        LOG_ERR("listen() failed: %d", errno);
        zsock_close(listen_sock);
        listen_sock = -1;
        return -errno;
    }

    /* Non-blocking accept so the emulator keeps running (and the input thread
     * stays responsive) even when no viewer is connected. */
    int flags = zsock_fcntl(listen_sock, ZVFS_F_GETFL, 0);
    zsock_fcntl(listen_sock, ZVFS_F_SETFL, flags | ZVFS_O_NONBLOCK);

    LOG_INF("TCP framebuffer server listening on port %d", LISTEN_PORT);
    return 0;
}

/* Send exactly len bytes, looping over partial writes. conn_sock is a blocking
 * socket, so zsock_send() only returns on progress or a real error — it parks
 * the thread until TX space frees instead of busy-polling. Returns 0 on success,
 * <0 if the connection broke. */
static int send_all(const uint8_t *buf, int len)
{
    int sent = 0;

    while (sent < len) {
        int n = zsock_send(conn_sock, buf + sent, len - sent, 0);
        if (n > 0) {
            sent += n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            /* Shouldn't happen on a blocking socket, but stay correct if it
             * does: yield rather than spin. */
            k_msleep(1);
            continue;
        }
        /* n == 0 or a real error: peer closed / connection reset. */
        return -1;
    }
    return 0;
}

void fb_udp_send_frame(const uint8_t *pixels, int width, int height)
{
    if (listen_sock < 0) {
        return;
    }

    /* No viewer connected yet — accept one (blocks until the PC connects). */
    if (conn_sock < 0) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);

        conn_sock = zsock_accept(listen_sock, (struct sockaddr *)&peer,
                                 &peer_len);
        if (conn_sock < 0) {
            return;     /* try again next frame */
        }

        /* listen_sock is non-blocking and the accepted socket may inherit that;
         * force the data socket blocking so send_all() parks instead of
         * spin-sleeping on EAGAIN (each k_msleep(1) cost a full tick/frame). */
        int f = zsock_fcntl(conn_sock, ZVFS_F_GETFL, 0);
        zsock_fcntl(conn_sock, ZVFS_F_SETFL, f & ~ZVFS_O_NONBLOCK);

        /* Disable Nagle: we already coalesce header+payload into a few large
         * sends, and Nagle would otherwise delay them waiting for ACKs. */
        int one = 1;
        zsock_setsockopt(conn_sock, IPPROTO_TCP, TCP_NODELAY,
                         &one, sizeof(one));

        LOG_INF("viewer connected");
    }

    uint32_t total_bytes = (uint32_t)width * height; /* 1 byte/pixel */

    /* 4-byte big-endian length header, sent in the same buffer as the start of
     * the payload would be — but since payload is a separate caller buffer we
     * just send the header first; TCP_NODELAY keeps it from stalling. */
    uint8_t hdr[LEN_HEADER_SIZE];
    hdr[0] = (total_bytes >> 24) & 0xFF;
    hdr[1] = (total_bytes >> 16) & 0xFF;
    hdr[2] = (total_bytes >> 8) & 0xFF;
    hdr[3] = total_bytes & 0xFF;

    if (send_all(hdr, LEN_HEADER_SIZE) < 0 ||
        send_all(pixels, total_bytes) < 0) {
        LOG_INF("viewer disconnected");
        zsock_close(conn_sock);
        conn_sock = -1;
    }
}
