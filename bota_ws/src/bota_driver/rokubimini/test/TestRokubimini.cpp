/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Reading
 */

#include <gtest/gtest.h>

#include <rokubimini/Rokubimini.hpp>

namespace rokubimini
{
// class RokubiminiTest : public ::testing::Test
// {
// protected:
//   std::vector<std::shared_ptr<rokubimini::Rokubimini>> rokubiminis;
//   rokubimini::Rokubimini* rokubimini;
//   const std::string setupYaml = ros::package::getPath("rokubimini") + "/test/setup.yaml";
//   RokubiminiTest() : rokubiminis()
//   {
//     rokubimini = new Rokubimini();
//   }

//   ~RokubiminiTest() override
//   {
//     delete rokubimini;
//     // You can do clean-up work that doesn't throw exceptions here.
//   }

//   // If the constructor and destructor are not enough for setting up
//   // and cleaning up each test, you can define the following methods:

//   void SetUp() override
//   {
//     // Code here will be called immediately after the constructor (right
//     // before each test).
//   }

//   void TearDown() override
//   {
//     // Code here will be called immediately after each test (right
//     // before the destructor).
//   }
// };

// TEST_F(RokubiminiTest, FromFileWorksCorrectly)
// {
//   rokubimini::setup::Setup* setup = new rokubimini::setup::Setup();
//   setup->fromFile(setupYaml);
//   for (const auto& rokubimini_setup : setup->rokubiminis_)
//   {
//     auto rokubimini = std::make_shared<rokubimini::Rokubimini>();
//     rokubimini->loadRokubiminiSetup(rokubimini_setup);
//     rokubiminis->emplace_back(rokubimini);
//   }
//   ASSERT_EQ(rokubiminis.empty(), false);
//   ASSERT_EQ(rokubiminis.size(), (uint32_t)4);

//   EXPECT_STREQ(rokubiminis[0]->getName().c_str(), "ROKUB_ETHERCAT_1");

//   EXPECT_STREQ(rokubiminis[1]->getName().c_str(), "ROKUB_SERIAL_USB0");

//   EXPECT_STREQ(rokubiminis[2]->getName().c_str(), "ROKUB_ETHERCAT_2");

//   EXPECT_STREQ(rokubiminis[3]->getName().c_str(), "ROKUB_SERIAL_ACM0");
// }

// TEST_F(RokubiminiTest, StatusWordWorksCorrectly)
// {
//   Statusword word = Statusword();
//   rokubimini->setStatusword(word);
//   auto word_expected = reading->getStatusword();
//   EXPECT_EQ(word.getData(), word_expected.getData());
//   EXPECT_EQ(word.getStamp(), word_expected.getStamp());
//   EXPECT_EQ(word.hasErrorAdcSaturated(), word_expected.hasErrorAdcSaturated());
//   EXPECT_EQ(word.hasErrorAccSaturated(), word_expected.hasErrorAccSaturated());
//   EXPECT_EQ(word.hasErrorGyroSaturated(), word_expected.hasErrorGyroSaturated());
//   EXPECT_EQ(word.hasErrorAdcOutOfSync(), word_expected.hasErrorAdcOutOfSync());
//   EXPECT_EQ(word.hasErrorSensingRangeExceeded(), word_expected.hasErrorSensingRangeExceeded());
//   EXPECT_EQ(word.hasWarningOvertemperature(), word_expected.hasWarningOvertemperature());
//   EXPECT_EQ(word.hasFatalSupplyVoltage(), word_expected.hasFatalSupplyVoltage());
// }

// TEST_F(RokubiminiTest, StagingCommandWorksCorrectly)
// {
//   Command new_command = Command();
//   rokubimini->stageCommand(new_command);
//   EXPECT_EQ(rokubimini->commandIsStaged(), true);
// }

// void customCb(const std::string& str)
// {
//   EXPECT_EQ(str.empty(), true);
//   return;
// }
// void customCb2(const std::string& str)
// {
//   EXPECT_EQ(str.empty(), true);
//   return;
// }
// TEST_F(RokubiminiTest, CallbacksWorkCorrectly)
// {
//   rokubimini->addErrorCb(customCb, 1);
//   rokubimini->addErrorCb(customCb2, 2);
// }
}  // namespace rokubimini
