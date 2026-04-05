#pragma once

#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

static constexpr size_t SERIAL_MAX_LINE = 4096;

inline auto open_serial(const char *path) -> int {
    int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "[serial] open('%s'): %s\n", path, strerror(errno));
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[serial] tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= static_cast<tcflag_t>(CS8);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag |= static_cast<tcflag_t>(CREAD | CLOCAL);
    tty.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO | ECHOE | ISIG));
    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    tty.c_oflag &= static_cast<tcflag_t>(~OPOST);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[serial] tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

// Reads bytes until '\n' or timeout_ms elapses. Strips trailing '\r'.
inline auto read_line(int fd, std::string &out, int timeout_ms) -> bool {
    out.clear();
    out.reserve(512);

    struct timeval deadline{};
    gettimeofday(&deadline, nullptr);
    deadline.tv_usec += timeout_ms * 1000L;
    deadline.tv_sec += deadline.tv_usec / 1'000'000L;
    deadline.tv_usec %= 1'000'000L;

    while (out.size() < SERIAL_MAX_LINE) {
        struct timeval now{};
        gettimeofday(&now, nullptr);
        long rem_us =
            ((deadline.tv_sec - now.tv_sec) * 1'000'000L) + (deadline.tv_usec - now.tv_usec);
        if (rem_us <= 0) {
            break;
        }

        struct timeval tv{};
        tv.tv_sec  = rem_us / 1'000'000L;
        tv.tv_usec = rem_us % 1'000'000L;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
            break;
        }

        char c = 0;
        if (read(fd, &c, 1) <= 0) {
            continue;
        }
        if (c == '\n') {
            if (!out.empty() && out.back() == '\r') {
                out.pop_back();
            }
            return !out.empty();
        }
        out.push_back(c);
    }

    if (!out.empty() && out.back() == '\r') {
        out.pop_back();
    }
    return !out.empty();
}

// Reads and discards all pending data until the bus stays silent for quiet_ms.
inline void drain_until_quiet(int fd, int quiet_ms) {
    char discard[64];
    while (true) {
        struct timeval tv{.tv_sec = 0, .tv_usec = quiet_ms * 1000L};
        fd_set         rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
            break;
        }
        read(fd, discard, sizeof(discard));
    }
}

// Send a single one-byte command (space, !, ").
inline auto send_command(int fd, char cmd) -> bool {
    if (write(fd, &cmd, 1) != 1) {
        fprintf(stderr, "[serial] write 0x%02X\n", static_cast<unsigned char>(cmd));
        strerror(errno);
        return false;
    }
    tcdrain(fd);
    return true;
}

// Send one byte of a multi-byte & command and wait for the controller's echo.
inline auto send_byte_wait_echo(int fd, char c, int timeout_ms = 200) -> bool {
    if (write(fd, &c, 1) != 1) {
        return false;
    }
    tcdrain(fd);

    struct timeval deadline{};
    gettimeofday(&deadline, nullptr);
    deadline.tv_usec += timeout_ms * 1000L;
    deadline.tv_sec += deadline.tv_usec / 1'000'000L;
    deadline.tv_usec %= 1'000'000L;

    while (true) {
        struct timeval now{};
        gettimeofday(&now, nullptr);
        long rem_us =
            ((deadline.tv_sec - now.tv_sec) * 1'000'000L) + (deadline.tv_usec - now.tv_usec);
        if (rem_us <= 0) {
            return false;
        }

        struct timeval tv{};
        tv.tv_sec  = rem_us / 1'000'000L;
        tv.tv_usec = rem_us % 1'000'000L;
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
            return false;
        }

        char echo = 0;
        if (read(fd, &echo, 1) > 0) {
            if (echo == c) {
                return true;
            }
            if (echo == '?') {
                fprintf(stderr,
                        "[serial] controller rejected byte 0x%02X\n",
                        static_cast<unsigned char>(c));
                return false;
            }
        }
    }
}

// Send a complete &-command string one byte at a time, waiting for each echo.
// e.g. "&GAA0500", "&H1C01"
inline auto send_ampersand_command(int fd, const char *cmd) -> bool {
    for (int i = 0; cmd[i] != '\0'; ++i) {
        if (!send_byte_wait_echo(fd, cmd[i])) {
            fprintf(stderr, "[serial] echo timeout at byte %d of '%s'\n", i, cmd);
            return false;
        }
    }
    return true;
}
