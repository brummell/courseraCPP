// Convert this program to C++ - CHECK
// change to C++ io - CHECK
// change to one line comments - CHECK
// change defines of constants to const - CHECK
// change array to vector<> - CHECK
// inline any short function - CHECK

#include <iostream>
#include <vector>

using namespace std;

template <typename T>
inline T sum(const vector<T>& summable) {
    T accum = 0;
    for (const auto& i : summable) {
        accum += i;
    }
    return accum;
}

int main() {
    const int N {40};

    vector<int> data;
    data.reserve(N);

    for (int i {0}; i < N; ++i) {
        data.push_back(i);
    }

    auto accum = sum(data);

    cout << "sum is " << accum << endl;

    return 0;
}