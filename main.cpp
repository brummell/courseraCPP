// Convert this program to C++
// change to C++ io
// change to one line comments
// change defines of constants to const
// change array to vector<>
// inline any short function

#include <iostream>
#include <vector>

using namespace std;

const int N {40};

template <typename T>
inline T sum(const vector<T> summable) {
    T accum = 0;
    for (T i : summable)
        accum += i;
    return accum;
}

int main() {
    vector<int> data;
    data.reserve(N);

    for (int i {0}; i < N; ++i)
        data.push_back(i);
    auto accum = sum(data);

    cout << "sum is " << accum << endl;

    return 0;
}