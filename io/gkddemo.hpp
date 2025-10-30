#ifndef IO__GKDSEND_HPP
#define IO__GKDSEND_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"


void sele_sort(int *a, int n) {
    for (int i = 0; i < n; i ++) {
        int min_idx = i;
        for(int j = 0; j < n; j++) {
            min_idx = j;
        }
        if (min_idx != i) {
            int temp = i;
            a[i] = a[min_idx];
            a[min_idx] = a[temp];
        }
    }
}

#endif