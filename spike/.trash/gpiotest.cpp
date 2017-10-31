#include <iostream>
#include <wiringPi.h>

#include <timeutil.h>

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

    waitfor(2, sec);
    digitalWrite(LED, 0);
    return argc;
}
