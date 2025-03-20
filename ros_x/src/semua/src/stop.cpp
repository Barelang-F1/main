#include <signal.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cstdlib>  // Untuk popen
#include <unistd.h> // Untuk sleep

#include <JetsonGPIO.h>

using namespace std;

static bool end_this_program = false;
static bool is_first_press = true; // Untuk melacak status tombol

inline void delay(int s) { this_thread::sleep_for(chrono::seconds(s)); }

// Fungsi untuk menjalankan perintah shell
void executeCommand(const string& command) {
    int ret_code = system(command.c_str());
    if (ret_code != 0) {
        cerr << "Gagal menjalankan perintah: " << command << endl;
    }
}

// Fungsi untuk menghentikan rosnode tertentu
void stopRosNode(const string& node_name) {
    FILE *fp;
    char path[1035];

    // Perintah untuk mencari proses rosnode
    string command = "pgrep -f " + node_name;
    fp = popen(command.c_str(), "r");
    if (fp == NULL) {
        cerr << "Tidak bisa menjalankan perintah pgrep!" << endl;
        return;
    }

    // Membaca PID dari output pgrep
    while (fgets(path, sizeof(path) - 1, fp) != NULL) {
        int pid = atoi(path);  // Konversi PID dari string ke integer
        if (pid > 0) {
            // Mengirim sinyal TERM untuk menghentikan proses
            kill(pid, SIGTERM);
            cout << "Proses rosnode " << node_name << " dengan PID " << pid << " dihentikan." << endl;
        }
    }

    fclose(fp);
}

void signalHandler(int s) { end_this_program = true; }

int main()
{
    // Ketika CTRL+C ditekan, signalHandler akan dipanggil
    signal(SIGINT, signalHandler);

    // Definisi Pin
    int led_pin = 13;  // Pin LED (Pin 13)
    int but_pin = 6;  // Pin Tombol (Pin 12)

    // Pengaturan Pin.
    GPIO::setmode(GPIO::BCM);

    // Set pin sebagai output dengan status awal LOW
    GPIO::setup(led_pin, GPIO::OUT, GPIO::LOW);
    GPIO::setup(but_pin, GPIO::IN);

    cout << "Demo dimulai sekarang! Tekan CTRL+C untuk keluar" << endl;

    while (!end_this_program) {
    cout << "Menunggu event tombol" << endl;
    bool event_detected = GPIO::wait_for_edge(but_pin, GPIO::Edge::FALLING, 1000); // Timeout 1 detik

    if (end_this_program) {
        break; // Periksa apakah program harus dihentikan
    }

    if (event_detected) {
        // Event diterima ketika tombol ditekan
        cout << "Tombol Ditekan!" << endl;

        if (is_first_press) {
            cout << "Memublikasikan data dari file bno055" << endl;
            executeCommand("rosrun semua bno055_imu &");
            executeCommand("rosrun semua sub &");

            GPIO::output(led_pin, GPIO::HIGH);
            delay(1);
            GPIO::output(led_pin, GPIO::LOW);

            is_first_press = false;
        } else {
            cout << "Mematikan rosnode bno055_imu dan sub" << endl;
            stopRosNode("bno055_imu");
            stopRosNode("sub");

            GPIO::output(led_pin, GPIO::HIGH);
            delay(1);
            GPIO::output(led_pin, GPIO::LOW);

            is_first_press = true;
        }
    }
}

    GPIO::cleanup();

    return 0;
}
