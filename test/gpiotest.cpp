#include <wiringPi.h>
#include <TimeUtil.h>
#include <iostream>

#define LED 1

using namespace std;

int main(int argc, char** argv)
{
    wiringPiSetup();
    pinMode(LED, 1);
    cout << "Light on...\n";
    digitalWrite(LED, 1);

    for (int r = 0; r < 2; r++)
    {
        for (int i = 0; i < 26; i++)
        {
            if (i % 2 != r) cout << i << " ";
        }
        cout << endl;
    }

    waitFor(2, SEC);
    digitalWrite(LED, 0);
    return argc;
}
