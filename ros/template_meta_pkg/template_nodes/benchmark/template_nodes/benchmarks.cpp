/**
 * @file
 * @brief Benchmark examples
 *
 * @author Henrik Stickann
 */

// IGMR library headers
#include <template_nodes/example_library/example_class.h>

// Standard library headers
#include <string>

// Benchmark library header
#include <benchmark/benchmark.h>


using namespace template_nodes;


/**
 * @brief Benchmark example testing string creation time
 */
static void StringCreation(benchmark::State & state)
{
    // Code inside this loop is measured repeatedly
    for (auto _ : state) {
        std::string created_string("hello");
        // Make sure the variable is not optimized away by compiler
        benchmark::DoNotOptimize(created_string);
    }
}

/**
 * @brief Benchmark example testing string copy time
 */
static void StringCopy(benchmark::State & state)
{
    // Code before the loop is not measured
    std::string x = "hello";
    for (auto _ : state) {
        std::string copy(x);
    }
}


// Register the functions as a benchmark
BENCHMARK(StringCreation);
BENCHMARK(StringCopy);


BENCHMARK_MAIN();
