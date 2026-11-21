#ifndef _FIFO_h
#define _FIFO_h
#define FIFO_WIPE 1
template <typename T, uint16_t SIZE >
class FIFO {
public:
    FIFO() {
        for (uint16_t i = 0; i < SIZE; i++) buf[i] = 0;
    }
    void clear() {
        head = tail = 0;
    }
    uint16_t available() {
        return (SIZE + head - tail) % SIZE;
    }
    T get(int16_t i = 0) {
        return buf[(SIZE + tail + i) % SIZE];
    }
    T getLast() {
        return buf[(SIZE + head - 1) % SIZE];
    }
    void add(T data) {
        uint16_t i = (head + 1) % SIZE;
        if (i != tail) {
            buf[head] = data;
            head = i;
        }
    }
    void set(int16_t i, T data) {
        buf[(SIZE + tail + i) % SIZE] = data;
    }
    void next(bool wipe = 0) {
        if (wipe) buf[tail] = 0;
        tail = (tail + 1) % SIZE;
    }
    bool availableForWrite() {
        return ((head + 1) % SIZE != tail);
    }
private:
    T buf[SIZE];
    uint16_t head = 0, tail = 0;
};
#endif