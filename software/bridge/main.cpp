#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <mutex>
#include <thread>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

struct StatisticsData {
    size_t rx_msgs = 0;
    size_t tx_msgs = 0;
    size_t rx_bytes = 0;
    size_t tx_bytes = 0;
    std::mutex mutex;
};

void receiver(StatisticsData &stats) {
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in src_addr{};
    src_addr.sin_family = AF_INET;
    src_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    src_addr.sin_port = htons(4444);
    bind(sock, (struct sockaddr *)&src_addr, sizeof(src_addr));

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind("tcp://localhost:6000");

    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    std::vector<uint8_t> buffer(65536);

    while(1) {
        const ssize_t size = recvfrom(sock, buffer.data(), buffer.size(), 0,
                                      reinterpret_cast<sockaddr *>(&addr), &addr_len);
        if(size <= 0) {
            continue;
        }

        {
            std::lock_guard lock(stats.mutex);
            stats.rx_msgs++;
            stats.rx_bytes += size;
        }

        try {
            const nlohmann::json json =
                nlohmann::json::from_msgpack(buffer.begin(), buffer.begin() + size);
            const std::string str = json.dump();
            zmq::message_t message(str.begin(), str.end());
            publisher.send(message, zmq::send_flags::none);
        } catch(const std::exception &e) {
            std::cerr << "MessagePack decode error: " << e.what() << std::endl;
        }
    }

    publisher.close();
    close(sock);
}

void transmitter(StatisticsData &stats) {
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in send_addr{};
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr("10.42.0.19");
    send_addr.sin_port = htons(3333);

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.bind("tcp://localhost:7000");
    subscriber.set(zmq::sockopt::subscribe, "");

    while(1) {
        zmq::message_t message;
        auto result = subscriber.recv(message, zmq::recv_flags::none);
        if(!result || message.size() == 0) {
            continue;
        }

        try {
            const std::string str(static_cast<char *>(message.data()), message.size());
            const nlohmann::json json = nlohmann::json::parse(str);
            const std::vector<uint8_t> buffer = nlohmann::json::to_msgpack(json);

            {
                std::lock_guard lock(stats.mutex);
                stats.tx_msgs++;
                stats.tx_bytes += buffer.size();
            }

            sendto(sock, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&send_addr),
                   sizeof(send_addr));
        } catch(const std::exception &e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
        }
    }

    subscriber.close();
    close(sock);
}

void statistics(StatisticsData &stats) {
    std::cout << "\033[2J\033[H";
    std::cout << std::setprecision(3) << std::fixed;

    while(true) {
        {
            std::lock_guard lock(stats.mutex);

            const double upload = stats.tx_bytes / 1024.;
            const double download = stats.rx_bytes / 1024.;

            std::cout << "\0337";
            std::cout << "\033[1;1H";
            std::cout << "\033[K";
            std::cout << "------------------------------------------------\n";
            std::cout << "  upload: " << std::setw(7) << upload << " kB/s " << std::setw(3)
                      << stats.tx_msgs << " msg/s\n";
            std::cout << "download: " << std::setw(7) << download << " kB/s " << std::setw(3)
                      << stats.rx_msgs << " msg/s\n";
            std::cout << "------------------------------------------------\n";
            std::cout << "\0338";
            std::cout.flush();

            stats.rx_msgs = 0;
            stats.tx_msgs = 0;
            stats.rx_bytes = 0;
            stats.tx_bytes = 0;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    StatisticsData stats;

    std::thread thread_receiver(receiver, std::ref(stats));
    std::thread thread_transmitter(transmitter, std::ref(stats));
    std::thread thread_statistics(statistics, std::ref(stats));

    thread_receiver.join();
    thread_transmitter.join();
    thread_statistics.join();
}
