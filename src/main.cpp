#include "encoder_to_odom/encoder_converter.h"


int main(int argc, char* argv[])
{
    // Create Converter object
    rclcpp::init(argc, argv);
    // auto converter = ENCODER_CONVERTER::Converter();
    rclcpp::spin(std::make_shared<ENCODER_CONVERTER::Converter>());
    rclcpp::shutdown();
    return 0;
}