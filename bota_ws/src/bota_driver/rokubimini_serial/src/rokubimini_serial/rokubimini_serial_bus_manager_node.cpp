#include <rokubimini_serial/RokubiminiSerialBusManager.hpp>

int main(int argc, char** argv)
{
  bota_node::Nodewrap<rokubimini::serial::RokubiminiSerialBusManager> node(argc, argv, "bota_device_driver", 2, true);
  node.execute();
  return 0;
}
