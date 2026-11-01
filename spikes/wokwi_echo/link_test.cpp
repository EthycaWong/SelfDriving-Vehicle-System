// Connects to Wokwi's RFC2217 serial server, sends numbered lines and reports what comes back.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <string>

constexpr unsigned char IAC = 255, DONT = 254, DO = 253, WONT = 252, WILL = 251, SB = 250, SE = 240;

int main(int argc, char** argv) {
    int port = argc > 1 ? std::stoi(argv[1]) : 4000;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::perror("connect");
        return 2;
    }
    std::printf("connected to port %d\n", port);

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    auto lastSend = start;
    int sent = 0, echoed = 0, beats = 0, negotiation = 0;
    std::string line;
    enum { DATA, CMD, OPT, SUB, SUB_IAC } state = DATA;
    unsigned char verb = 0;

    while (clock::now() - start < std::chrono::seconds(10)) {
        if (clock::now() - lastSend > std::chrono::milliseconds(200)) {
            lastSend = clock::now();
            std::string msg = "PING " + std::to_string(sent++) + "\n";
            send(fd, msg.data(), msg.size(), 0);
        }
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(fd, &rd);
        timeval tv{0, 50000};
        if (select(fd + 1, &rd, nullptr, nullptr, &tv) <= 0) continue;
        unsigned char buf[512];
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) { std::puts("connection closed"); break; }
        for (ssize_t i = 0; i < n; ++i) {
            unsigned char c = buf[i];
            switch (state) {
                case DATA:
                    if (c == IAC) { state = CMD; break; }
                    if (c == '\n') {
                        if (line.rfind("PING", 0) == 0) ++echoed;
                        if (line.rfind("BEAT", 0) == 0) ++beats;
                        std::printf("< %s\n", line.c_str());
                        line.clear();
                    } else if (c != '\r') {
                        line.push_back(static_cast<char>(c));
                    }
                    break;
                case CMD:
                    if (c == IAC) { line.push_back(static_cast<char>(c)); state = DATA; }
                    else if (c == SB) state = SUB;
                    else if (c == WILL || c == WONT || c == DO || c == DONT) { verb = c; state = OPT; }
                    else state = DATA;
                    break;
                case OPT: {
                    ++negotiation;
                    unsigned char reply[3] = {IAC, static_cast<unsigned char>(verb == WILL ? DO : verb == DO ? WILL : verb == WONT ? DONT : WONT), c};
                    send(fd, reply, 3, 0);
                    state = DATA;
                    break;
                }
                case SUB: if (c == IAC) state = SUB_IAC; break;
                case SUB_IAC: state = c == SE ? DATA : SUB; break;
            }
        }
    }
    close(fd);
    std::printf("sent=%d echoed=%d beats=%d telnet_negotiations=%d\n", sent, echoed, beats, negotiation);
    return (echoed > 0 && beats > 0) ? 0 : 1;
}
