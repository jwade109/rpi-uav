#include <BNO055.h>
#include <iostream>
#include <stdio.h>
#include <TimeUtil.h>

using namespace std;

int main()
{
    BNO055 bno;

    bool status = bno.begin(0x28);
    if (status)
    {
        for (uint64_t i = 0; i < 100000; i++)
        {
            imu::Vector<3> q = bno.getVector(VECTOR_EULER);
            printf("%lf\t%lf\t%lf\n", q.x(), q.y(), q.z());
            waitFor(100, MILLI);
        }
    }
    else if (status & 0) cout << "Init BNO055" << endl;
    else        cout << "No BNO055 found." << endl;
    return 0;
}
