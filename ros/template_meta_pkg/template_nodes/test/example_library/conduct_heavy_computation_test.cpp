/**
 * @file
 * @brief Conduct heavy computation test
 *
 * @author Henrik Stickann
 */

// IGMR library header
#include <template_nodes/example_library/example_class.h>

// GTest header
#include <gtest/gtest.h>


using namespace template_nodes;


/**
 * @brief Test if output equals 0 if input is 0
 */
TEST(MathOperation, testZero)
{
    template_nodes::ExampleClass sim_ex;
    sim_ex.setInput(0);
    sim_ex.conductHeavyComputation();
    int result = sim_ex.getResult();

    EXPECT_EQ(result, 0);
}

/**
 * @brief Test if output equals 6 if input is 3
 */
TEST(MathOperation, testSimple)
{
    template_nodes::ExampleClass sim_ex;
    sim_ex.setInput(3);
    sim_ex.conductHeavyComputation();
    int result = sim_ex.getResult();

    EXPECT_EQ(result, 6);
}


/**
 * @brief Entry point for GTest framework
 */
int main(int argc, char ** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
