/**
 * @file
 * @brief Test template class
 *
 * @author Henrik Stickann
 */

// IGMR library header
#include <template_nodes/example_templates/example_templates.h>

// GTest header
#include <gtest/gtest.h>


using namespace template_nodes;


/**
 * @brief Test floating-point addition
 */
TEST(TemplateClassTests, testAddFloat)
{
    template_nodes::TemplateClass<float> impl;
    float result = impl.add(1.3, 0.7);

    EXPECT_FLOAT_EQ(result, 2.0);
}


/**
 * @brief Entry point for GTest framework
 */
int main(int argc, char ** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
