#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <cstdio>  // For popen()

using namespace std;

// Function to generate the disturbance signal
double disturbance(double t) {
    if (t < 3.2) {
        return 0;  // No disturbance
    } else if (t < 9.4) {
        return 5 * sin(2 * M_PI * (t - 3.2) / (9.4 - 3.2));  // Sinusoidal disturbance
    } else if (t < 14) {
        return (fmod(t - 9.7, 1.0) < 0.5) ? 5 : 0;  // Pulse disturbance
    } else if (t <= 16) {
        return (t - 14) * 2.5;  // Ramp disturbance
    } else {
        return 0;  // No disturbance after 16s
    }
}

int main() {
    // Step 1: Generate data file
    ofstream file("data.txt");
    if (!file) {
        cerr << "Error: Could not create data file!" << endl;
        return 1;
    }

    for (double t = 0; t <= 16; t += 0.1) {
        file << t << " " << disturbance(t) << endl;
    }
    file.close();
    cout << "Data saved to data.txt\n";

    // Step 2: Generate Gnuplot script
    ofstream gnuplotFile("plot.gnu");
    if (!gnuplotFile) {
        cerr << "Error: Could not create Gnuplot script file!" << endl;
        return 1;
    }

    gnuplotFile << "set terminal png\n";
    gnuplotFile << "set output 'disturbance_plot.png'\n";
    gnuplotFile << "set title 'Disturbance Signal'\n";
    gnuplotFile << "set xlabel 'Time (s)'\n";
    gnuplotFile << "set ylabel 'Disturbance'\n";
    gnuplotFile << "plot 'data.txt' using 1:2 with lines title 'Disturbance'\n";
    gnuplotFile.close();

    // Step 3: Run Gnuplot command
    cout << "Generating plot...\n";
    system("gnuplot plot.gnu");
    cout << "Plot saved as disturbance_plot.png\n";

    return 0;
}
