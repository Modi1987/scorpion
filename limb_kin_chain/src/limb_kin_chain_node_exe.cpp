#include "limb_kin_chain/limb_kin_chain.hpp"
#include "limb_kin_chain/limb_kin_chain_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    penta_pod::kin::limb_kin_chain::Limb limb;
    std::unique_ptr<penta_pod::kin::limb_kin_chain::LimbNode> limbNode(new penta_pod::kin::limb_kin_chain::LimbNode(limb));
    limbNode->spin();
    rclcpp::shutdown();
    return 0;
}