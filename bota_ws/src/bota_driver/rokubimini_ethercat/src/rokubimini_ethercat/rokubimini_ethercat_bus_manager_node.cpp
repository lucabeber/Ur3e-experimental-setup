#include <rokubimini_ethercat/RokubiminiEthercatBusManager.hpp>

int main(int argc, char** argv)
{
  bota_node::Nodewrap<rokubimini::ethercat::RokubiminiEthercatBusManager> node(argc, argv, "bota_device_driver", 2,
                                                                               true);
  node.execute();
  return 0;
}
