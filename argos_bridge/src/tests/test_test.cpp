# include <vector>

int sum_integers(const std::vector<int> integers) {
    int sum = 0;
    for (int i : integers) {
        sum += i;
    }
    return sum;
}

int main() {
    std::vector<int> integers = {1, 2, 3, 4, 5};

    if (sum_integers(integers) == 15) {
        return 0;
    } else {
        return 1;
    }
}