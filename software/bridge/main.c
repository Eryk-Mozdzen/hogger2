#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>
#include <zmq.h>

void *udp_receiver(void *args) {
    (void)args;

    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(4444);
    bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    void *context = zmq_ctx_new();
    void *publisher = zmq_socket(context, ZMQ_PUB);
    zmq_bind(publisher, "tcp://localhost:6000");

    char buffer[65536];
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    while(1) {
        const ssize_t len =
            recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&addr, &addr_len);

        zmq_send(publisher, buffer, len, 0);
    }

    zmq_close(publisher);
    close(sock);

    return NULL;
}

void *udp_transmitter(void *args) {
    (void)args;

    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in send_addr = {0};
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr("192.168.4.1");
    send_addr.sin_port = htons(3333);

    void *context = zmq_ctx_new();
    void *subscriber = zmq_socket(context, ZMQ_SUB);
    zmq_bind(subscriber, "tcp://localhost:7000");
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    char buffer[65536];

    while(1) {
        const int len = zmq_recv(subscriber, buffer, sizeof(buffer), 0);

        sendto(sock, buffer, len, 0, (struct sockaddr *)&send_addr, sizeof(send_addr));
    }

    zmq_close(subscriber);
    close(sock);

    return NULL;
}

int main() {
    pthread_t receiver;
    pthread_t transmitter;

    pthread_create(&receiver, NULL, udp_receiver, NULL);
    pthread_create(&transmitter, NULL, udp_transmitter, NULL);

    pthread_join(receiver, NULL);
    pthread_join(transmitter, NULL);
}
